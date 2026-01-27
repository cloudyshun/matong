/*
 * STM32F103C8T6 步进电机控制系统 (74HC595 + ULN2803版)
 * 
 * 功能更新：
 * 1. 上电自动正转一圈。
 * 2. 指令① (EE...01...00): 电机1反转，电机2正转。 <--- 已修改
 * 3. 指令② (EE...02...00): 电机1反转，电机2反转。 <--- 保持不变
 * 
 * 硬件连接:
 * - 74HC595: PA5(Data), PA6(Latch), PA7(Clock)
 * - RS485: PA8(Control), PA9/PA10(UART)
 * - 电机1: PB口 (数据低4位) -> 对应 74HC595 QA-QD
 * - 电机2: PA口 (数据高4位) -> 对应 74HC595 QE-QH
 */

#include <Arduino.h>

// ================= 引脚定义 =================

// RS485控制引脚 (PA8)
#define DE_RE_Pin PA8

// 74HC595 控制引脚
#define PIN_595_DATA   PA5  // DS
#define PIN_595_LATCH  PA6  // STCP
#define PIN_595_CLOCK  PA7  // SHCP

// ================= 参数设置 =================
#define STEPS_PER_REVOLUTION 500  // 每圈步数
#define STEP_DELAY 5              // 步进速度(ms)

// ================= 全局变量 =================

// 缓存当前发送给 74HC595 的 8位数据
byte currentShiftOutput = 0; 
byte motor1Nibble = 0; // 电机1的当前4位状态
byte motor2Nibble = 0; // 电机2的当前4位状态

// 电机1 状态变量
bool motor1Running = false;
int motor1StepPhase = 0;
int motor1StepCycle = 0;
int motor1TargetSteps = 0;
unsigned long motor1LastStepTime = 0;
bool motor1Direction = true;  // true=正转, false=反转
bool motor1Enabled = false;   // 运行使能标志
int motor1RevolutionCount = 0;

// 电机2 状态变量
bool motor2Running = false;
int motor2StepPhase = 0;
int motor2StepCycle = 0;
int motor2TargetSteps = 0;
unsigned long motor2LastStepTime = 0;
bool motor2Direction = true;  // true=正转, false=反转
bool motor2Enabled = false;   // 运行使能标志
int motor2RevolutionCount = 0;

// ============ RS485控制指令定义 (HEX) ============

// 单独控制指令
const byte motor1OnCmd[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00}; // M1 单独正转
const byte motor1OffCmd[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00}; // M1 停止
const byte motor2OnCmd[8]  = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00}; // M2 单独正转
const byte motor2OffCmd[8] = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00}; // M2 停止

// 【组合控制指令】
// 指令①: 01号组合指令
const byte cmdAction1[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
// 指令②: 02号组合指令
const byte cmdAction2[8]  = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};

// RS485接收缓冲区
byte rs485Buffer[8];
int rs485Index = 0;

// 双相激励序列表 (HEX格式)
const byte stepPatternHex[4] = {0x0C, 0x06, 0x03, 0x09};
const byte stepPatternHexRev[4] = {0x09, 0x03, 0x06, 0x0C};

// 函数声明
void updateShiftRegister();
void stopMotor1();
void stopMotor2();
void startMotor1();
void startMotor2();
void calculatePhaseMotor1();
void calculatePhaseMotor2();
void finishRevolutionMotor1();
void finishRevolutionMotor2();
void handleStepperMotor(unsigned long currentTime);
void handleRS485Commands();
void processHexCommand(byte cmd[8]);
void sendHex485(byte data[8]);

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("STM32 74HC595 Motor Control Started...");

  // 初始化RS485控制脚 (PA8)
  pinMode(DE_RE_Pin, OUTPUT);
  digitalWrite(DE_RE_Pin, LOW); // 默认接收模式
  Serial1.begin(9600);
  
  // 初始化 74HC595 引脚
  pinMode(PIN_595_DATA, OUTPUT);
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_595_CLOCK, OUTPUT);

  // 初始清空输出
  stopMotor1();
  stopMotor2();
  updateShiftRegister(); 
  
  Serial.println("System Ready");

  // 初始化时间戳
  unsigned long currentTime = millis();
  motor1LastStepTime = currentTime;
  motor2LastStepTime = currentTime;

  // ========== 上电默认行为 ==========
  // 目前保持上电全正转。如果需要改成别的，请告诉我。
  motor1Direction = true; // 正转
  motor2Direction = true; // 正转
  motor1Enabled = true;
  motor2Enabled = true;
  
  Serial.println("Auto Start: All Forward");
}

void loop() {
  // 任务1: 步进电机逻辑
  handleStepperMotor(millis());

  // 任务2: RS485指令处理
  handleRS485Commands();
}

// ================= 核心控制逻辑 =================

// 将数据发送到 74HC595
void updateShiftRegister() {
  // 拼接数据：高4位是电机2，低4位是电机1
  currentShiftOutput = (motor2Nibble << 4) | (motor1Nibble & 0x0F);

  digitalWrite(PIN_595_LATCH, LOW); 
  shiftOut(PIN_595_DATA, PIN_595_CLOCK, LSBFIRST, currentShiftOutput);
  digitalWrite(PIN_595_LATCH, HIGH); 
}

void handleStepperMotor(unsigned long currentTime) {
  // --- 电机 1 逻辑 ---
  if(!motor1Running && motor1Enabled) {
    startMotor1();
  }

  if(motor1Running && (currentTime - motor1LastStepTime >= STEP_DELAY)) { 
    calculatePhaseMotor1(); 
    motor1StepPhase++;

    if(motor1StepPhase >= 4) {
      motor1StepPhase = 0;
      motor1StepCycle++;
      // 检查是否完成一圈
      if(motor1StepCycle >= motor1TargetSteps) {
        finishRevolutionMotor1();
      }
    }
    motor1LastStepTime = currentTime;
    updateShiftRegister(); 
  }

  // --- 电机 2 逻辑 ---
  if(!motor2Running && motor2Enabled) {
    startMotor2();
  }

  if(motor2Running && (currentTime - motor2LastStepTime >= STEP_DELAY)) { 
    calculatePhaseMotor2(); 
    motor2StepPhase++;

    if(motor2StepPhase >= 4) {
      motor2StepPhase = 0;
      motor2StepCycle++;
      // 检查是否完成一圈
      if(motor2StepCycle >= motor2TargetSteps) {
        finishRevolutionMotor2();
      }
    }
    motor2LastStepTime = currentTime;
    updateShiftRegister(); 
  }
}

// 计算电机1的相位数据
void calculatePhaseMotor1() {
  if (motor1Direction) {
    motor1Nibble = stepPatternHex[motor1StepPhase];
  } else {
    motor1Nibble = stepPatternHexRev[motor1StepPhase];
  }
}

// 计算电机2的相位数据
void calculatePhaseMotor2() {
  if (motor2Direction) {
    motor2Nibble = stepPatternHex[motor2StepPhase];
  } else {
    motor2Nibble = stepPatternHexRev[motor2StepPhase];
  }
}

void startMotor1() {
  if(motor1Running) return; 
  motor1RevolutionCount++;
  motor1StepCycle = 0;
  motor1StepPhase = 0;
  motor1TargetSteps = STEPS_PER_REVOLUTION;
  motor1Running = true;
}

void startMotor2() {
  if(motor2Running) return; 
  motor2RevolutionCount++;
  motor2StepCycle = 0;
  motor2StepPhase = 0;
  motor2TargetSteps = STEPS_PER_REVOLUTION;
  motor2Running = true;
}

// 电机1 完成一圈
void finishRevolutionMotor1() {
  motor1Running = false;
  motor1Enabled = false; // 关闭使能
  stopMotor1();          // 断电
  updateShiftRegister(); 
}

// 电机2 完成一圈
void finishRevolutionMotor2() {
  motor2Running = false;
  motor2Enabled = false; // 关闭使能
  stopMotor2();          // 断电
  updateShiftRegister(); 
}

void stopMotor1() {
  motor1Nibble = 0x00; 
}

void stopMotor2() {
  motor2Nibble = 0x00; 
}

// ================= RS485 通信 =================

void sendHex485(byte data[8]) {
  digitalWrite(DE_RE_Pin, HIGH); // 发送模式
  delayMicroseconds(20);
  Serial1.write(data, 8);
  Serial1.flush();
  delayMicroseconds(20);
  digitalWrite(DE_RE_Pin, LOW); // 接收模式

  Serial.print("Sent: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(data[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

void handleRS485Commands() {
  while (Serial1.available()) {
    byte receivedByte = Serial1.read();
    if (receivedByte == 0xEE) {
      rs485Buffer[0] = receivedByte;
      rs485Index = 1;
      unsigned long startTime = millis();
      while (rs485Index < 8) {
        if (Serial1.available()) {
          rs485Buffer[rs485Index++] = Serial1.read();
        }
        if (millis() - startTime > 20) {
          rs485Index = 0;
          return;
        }
      }
      if (rs485Index == 8) {
        processHexCommand(rs485Buffer);
      }
      rs485Index = 0;
    }
  }
}

void processHexCommand(byte cmd[8]) {
  if (cmd[0] != 0xEE) return;

  // 1. 检查是否是 指令① (01 00 00 00)
  // 动作：M1 反转，M2 正转
  if (memcmp(cmd, cmdAction1, 8) == 0) {
    // 复位状态
    motor1Running = false; 
    motor2Running = false;
    
    // 设置方向
    motor1Direction = false; // M1 反转 (修改点)
    motor2Direction = true;  // M2 正转 (修改点)
    
    // 启动
    motor1Enabled = true;   
    motor2Enabled = true;   
    
    Serial.println("CMD 1: M1 Rev, M2 Fwd");
    sendHex485(cmd);
  }
  // 2. 检查是否是 指令② (02 00 00 00)
  // 动作：M1 反转，M2 反转
  else if (memcmp(cmd, cmdAction2, 8) == 0) {
    // 复位状态
    motor1Running = false;
    motor2Running = false;
    
    // 设置方向
    motor1Direction = false; // M1 反转
    motor2Direction = false; // M2 反转
    
    // 启动
    motor1Enabled = true;    
    motor2Enabled = true;    
    
    Serial.println("CMD 2: All Backward");
    sendHex485(cmd);
  }
  // 3. 原有单独控制指令 (单M1正转)
  else if (memcmp(cmd, motor1OnCmd, 8) == 0) {
    motor1Running = false;
    motor1Direction = true; // 正转
    motor1Enabled = true;
    Serial.println("CMD: M1 Forward");
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor1OffCmd, 8) == 0) {
    motor1Enabled = false;
    motor1Running = false;
    stopMotor1();
    updateShiftRegister(); 
    Serial.println("CMD: Stop M1");
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2OnCmd, 8) == 0) {
    motor2Running = false;
    motor2Direction = true; // 正转
    motor2Enabled = true;
    Serial.println("CMD: M2 Forward");
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2OffCmd, 8) == 0) {
    motor2Enabled = false;
    motor2Running = false;
    stopMotor2();
    updateShiftRegister(); 
    Serial.println("CMD: Stop M2");
    sendHex485(cmd);
  }
}