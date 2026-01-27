/*
 * STM32F103C8T6 步进电机控制系统 (74HC595 + ULN2803版)
 * 
 * 功能更新：
 * 1. 上电自动正转一圈。
 * 2. 指令① (01): M1反转，M2正转 (1圈)。
 * 3. 指令② (02): M1反转，M2反转 (1圈)。
 * 4. 指令③ (03): M1 M2先同时逆转1圈，随后M2单独正转0.4圈(200步)。
 * 5. 单独指令修改：原“停止”指令现改为“反转1圈”。
 * 
 * 硬件连接:
 * - 74HC595: PA5(Data), PA6(Latch), PA7(Clock)
 * - RS485: PA8(Control), PA9/PA10(UART)
 * - 电机1: PB口 (低4位)
 * - 电机2: PA口 (高4位)
 */

#include <Arduino.h>

// ================= 引脚定义 =================
#define DE_RE_Pin PA8

#define PIN_595_DATA   PA5
#define PIN_595_LATCH  PA6
#define PIN_595_CLOCK  PA7

// ================= 参数设置 =================
#define STEPS_PER_REVOLUTION 500  // 每圈步数
#define STEP_DELAY 5              // 步进速度(ms)

// ================= 全局变量 =================

// 缓存数据
byte currentShiftOutput = 0; 
byte motor1Nibble = 0; 
byte motor2Nibble = 0; 

// 可变的目标步数
int nextTargetSteps1 = STEPS_PER_REVOLUTION; 
int nextTargetSteps2 = STEPS_PER_REVOLUTION; 

// 指令3的状态机
int command3Status = 0; 

// 电机1 状态
bool motor1Running = false;
int motor1StepPhase = 0;
int motor1StepCycle = 0;
int motor1TargetSteps = 0;
unsigned long motor1LastStepTime = 0;
bool motor1Direction = true;  
bool motor1Enabled = false;   
int motor1RevolutionCount = 0;

// 电机2 状态
bool motor2Running = false;
int motor2StepPhase = 0;
int motor2StepCycle = 0;
int motor2TargetSteps = 0;
unsigned long motor2LastStepTime = 0;
bool motor2Direction = true;  
bool motor2Enabled = false;   
int motor2RevolutionCount = 0;

// ============ RS485控制指令定义 ============

// 单独控制
const byte motor1FwdCmd[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00}; // M1 正转
const byte motor1RevCmd[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00}; // M1 反转 (原停止)
const byte motor2FwdCmd[8] = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00}; // M2 正转
const byte motor2RevCmd[8] = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00}; // M2 反转 (原停止)

// 组合指令
const byte cmdAction1[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00}; // M1逆 M2正
const byte cmdAction2[8]  = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00}; // 全逆
const byte cmdAction3[8]  = {0xEE, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00}; // 复合动作

byte rs485Buffer[8];
int rs485Index = 0;

// 双相激励序列表
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
  Serial.println("STM32 Motor System Started");

  pinMode(DE_RE_Pin, OUTPUT);
  digitalWrite(DE_RE_Pin, LOW); 
  Serial1.begin(9600);
  
  pinMode(PIN_595_DATA, OUTPUT);
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_595_CLOCK, OUTPUT);

  stopMotor1();
  stopMotor2();
  updateShiftRegister(); 
  
  // ========== 上电默认：全正转1圈 ==========
  nextTargetSteps1 = STEPS_PER_REVOLUTION;
  nextTargetSteps2 = STEPS_PER_REVOLUTION;
  motor1Direction = true; 
  motor2Direction = true; 
  motor1Enabled = true;
  motor2Enabled = true;
  command3Status = 0; 
  
  Serial.println("Auto Start: All Forward");
}

void loop() {
  handleStepperMotor(millis());
  handleRS485Commands();
}

// ================= 核心控制逻辑 =================

void updateShiftRegister() {
  currentShiftOutput = (motor2Nibble << 4) | (motor1Nibble & 0x0F);
  digitalWrite(PIN_595_LATCH, LOW); 
  shiftOut(PIN_595_DATA, PIN_595_CLOCK, LSBFIRST, currentShiftOutput);
  digitalWrite(PIN_595_LATCH, HIGH); 
}

void handleStepperMotor(unsigned long currentTime) {
  // --- 电机 1 ---
  if(!motor1Running && motor1Enabled) {
    startMotor1();
  }
  if(motor1Running && (currentTime - motor1LastStepTime >= STEP_DELAY)) { 
    calculatePhaseMotor1(); 
    motor1StepPhase++;
    if(motor1StepPhase >= 4) {
      motor1StepPhase = 0;
      motor1StepCycle++;
      if(motor1StepCycle >= motor1TargetSteps) {
        finishRevolutionMotor1();
      }
    }
    motor1LastStepTime = currentTime;
    updateShiftRegister(); 
  }

  // --- 电机 2 ---
  if(!motor2Running && motor2Enabled) {
    startMotor2();
  }
  if(motor2Running && (currentTime - motor2LastStepTime >= STEP_DELAY)) { 
    calculatePhaseMotor2(); 
    motor2StepPhase++;
    if(motor2StepPhase >= 4) {
      motor2StepPhase = 0;
      motor2StepCycle++;
      if(motor2StepCycle >= motor2TargetSteps) {
        finishRevolutionMotor2();
      }
    }
    motor2LastStepTime = currentTime;
    updateShiftRegister(); 
  }
}

void calculatePhaseMotor1() {
  if (motor1Direction) motor1Nibble = stepPatternHex[motor1StepPhase];
  else motor1Nibble = stepPatternHexRev[motor1StepPhase];
}

void calculatePhaseMotor2() {
  if (motor2Direction) motor2Nibble = stepPatternHex[motor2StepPhase];
  else motor2Nibble = stepPatternHexRev[motor2StepPhase];
}

void startMotor1() {
  if(motor1Running) return; 
  motor1RevolutionCount++;
  motor1StepCycle = 0;
  motor1StepPhase = 0;
  motor1TargetSteps = nextTargetSteps1; 
  motor1Running = true;
}

void startMotor2() {
  if(motor2Running) return; 
  motor2RevolutionCount++;
  motor2StepCycle = 0;
  motor2StepPhase = 0;
  motor2TargetSteps = nextTargetSteps2; 
  motor2Running = true;
}

// 电机1 完成
void finishRevolutionMotor1() {
  motor1Running = false;
  motor1Enabled = false; 
  stopMotor1();          
  updateShiftRegister(); 
}

// 电机2 完成
void finishRevolutionMotor2() {
  motor2Running = false;
  
  // === 指令3特殊逻辑 ===
  if (command3Status == 1) {
    command3Status = 2; // 进入阶段二
    motor2Direction = true; // 正转
    nextTargetSteps2 = 200; // 0.4圈
    Serial.println("CMD3: Phase 2 Start");
    stopMotor2();
    updateShiftRegister();
    return; // 继续运行
  }
  
  if (command3Status == 2) {
      Serial.println("CMD3: Done");
      command3Status = 0; 
  }

  motor2Enabled = false; 
  stopMotor2();          
  updateShiftRegister(); 
}

void stopMotor1() { motor1Nibble = 0x00; }
void stopMotor2() { motor2Nibble = 0x00; }

// ================= RS485 通信 =================

void sendHex485(byte data[8]) {
  digitalWrite(DE_RE_Pin, HIGH);
  delayMicroseconds(20);
  Serial1.write(data, 8);
  Serial1.flush();
  delayMicroseconds(20);
  digitalWrite(DE_RE_Pin, LOW);
  Serial.println("Ack Sent");
}

void handleRS485Commands() {
  while (Serial1.available()) {
    byte receivedByte = Serial1.read();
    if (receivedByte == 0xEE) {
      rs485Buffer[0] = receivedByte;
      rs485Index = 1;
      unsigned long startTime = millis();
      while (rs485Index < 8) {
        if (Serial1.available()) rs485Buffer[rs485Index++] = Serial1.read();
        if (millis() - startTime > 20) { rs485Index = 0; return; }
      }
      if (rs485Index == 8) processHexCommand(rs485Buffer);
      rs485Index = 0;
    }
  }
}

void processHexCommand(byte cmd[8]) {
  if (cmd[0] != 0xEE) return;
  
  // 安全重置区
  command3Status = 0;
  nextTargetSteps1 = STEPS_PER_REVOLUTION; // 500
  nextTargetSteps2 = STEPS_PER_REVOLUTION; // 500

  // 指令①: M1逆 M2正
  if (memcmp(cmd, cmdAction1, 8) == 0) {
    motor1Running = false; motor2Running = false;
    motor1Direction = false; // 逆
    motor2Direction = true;  // 正
    motor1Enabled = true; motor2Enabled = true;
    sendHex485(cmd);
  }
  // 指令②: 全逆
  else if (memcmp(cmd, cmdAction2, 8) == 0) {
    motor1Running = false; motor2Running = false;
    motor1Direction = false; // 逆
    motor2Direction = false; // 逆
    motor1Enabled = true; motor2Enabled = true;
    sendHex485(cmd);
  }
  // 指令③: 复合动作
  else if (memcmp(cmd, cmdAction3, 8) == 0) {
    motor1Running = false; motor2Running = false;
    command3Status = 1;      
    motor1Direction = false; // 逆
    motor2Direction = false; // 逆
    nextTargetSteps1 = 500;
    nextTargetSteps2 = 500;
    motor1Enabled = true; motor2Enabled = true;
    sendHex485(cmd);
  }
  // M1 正转
  else if (memcmp(cmd, motor1FwdCmd, 8) == 0) {
    motor1Running = false;
    motor1Direction = true; // 正转
    motor1Enabled = true;
    Serial.println("CMD: M1 Forward 1 Rev");
    sendHex485(cmd);
  }
  // M1 反转 (原停止指令)
  else if (memcmp(cmd, motor1RevCmd, 8) == 0) {
    motor1Running = false;
    motor1Direction = false; // 反转
    motor1Enabled = true;    // 开启
    Serial.println("CMD: M1 Reverse 1 Rev");
    sendHex485(cmd);
  }
  // M2 正转
  else if (memcmp(cmd, motor2FwdCmd, 8) == 0) {
    motor2Running = false;
    motor2Direction = true; // 正转
    motor2Enabled = true;
    Serial.println("CMD: M2 Forward 1 Rev");
    sendHex485(cmd);
  }
  // M2 反转 (原停止指令)
  else if (memcmp(cmd, motor2RevCmd, 8) == 0) {
    motor2Running = false;
    motor2Direction = false; // 反转
    motor2Enabled = true;    // 开启
    Serial.println("CMD: M2 Reverse 1 Rev");
    sendHex485(cmd);
  }
}