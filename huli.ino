/*
 * STM32F103C8T6 步进电机控制系统 (74HC595 + ULN2803版)
 * 
 * 硬件连接 (基于新原理图):
 * ---------------------------------------------------------
 * [控制芯片] 74HC595 (U3)
 * - DS (SER)   -> PA5   (数据)
 * - STCP (RCLK)-> PA6   (锁存)
 * - SHCP (SRCLK)-> PA7  (时钟)
 * 
 * [驱动输出] ULN2803 (U1) -> 步进电机
 * - 74HC595 QA-QD -> ULN2803 I1-I4 -> J8 (电机 1) -> 对应数据低4位
 * - 74HC595 QE-QH -> ULN2803 I5-I8 -> J7 (电机 2) -> 对应数据高4位
 * ---------------------------------------------------------
 * - RS485 DE/RE -> PB9
 * - RS485 TX/RX -> PA9/PA10 (Serial1)
 */

#include <Arduino.h>

// ================= 引脚定义 =================

// RS485控制引脚
#define DE_RE_Pin PB9

// 74HC595 控制引脚
#define PIN_595_DATA   PA5  // DS
#define PIN_595_LATCH  PA6  // STCP
#define PIN_595_CLOCK  PA7  // SHCP

// ================= 参数设置 =================
#define STEPS_PER_REVOLUTION 500  // 每圈步数
#define STEP_DELAY 5              // 步进速度(ms) - 12V供电可适当改小

// ================= 全局变量 =================

// 缓存当前发送给 74HC595 的 8位数据
// 高4位存电机2，低4位存电机1
byte currentShiftOutput = 0; 
byte motor1Nibble = 0; // 电机1的当前4位状态 (0000-1111)
byte motor2Nibble = 0; // 电机2的当前4位状态 (0000-1111)

// 电机1 状态变量
bool motor1Running = false;
int motor1StepPhase = 0;
int motor1StepCycle = 0;
int motor1TargetSteps = 0;
unsigned long motor1LastStepTime = 0;
bool motor1Direction = true;  
bool motor1Enabled = false;   
int motor1RevolutionCount = 0;

// 电机2 状态变量
bool motor2Running = false;
int motor2StepPhase = 0;
int motor2StepCycle = 0;
int motor2TargetSteps = 0;
unsigned long motor2LastStepTime = 0;
bool motor2Direction = true; 
bool motor2Enabled = false;   
int motor2RevolutionCount = 0;

// RS485控制指令 (保持不变)
const byte motor1OnCmd[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00};
const byte motor1OffCmd[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00};
const byte motor2OnCmd[8]  = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00};
const byte motor2OffCmd[8] = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00};

// RS485接收缓冲区
byte rs485Buffer[8];
int rs485Index = 0;

// 双相激励序列表 (使用 HEX 格式方便移位操作)
// 对应二进制: 1100, 0110, 0011, 1001
const byte stepPatternHex[4] = {0x0C, 0x06, 0x03, 0x09};

// 反转序列
// 对应二进制: 1001, 0011, 0110, 1100
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

  // 初始化RS485
  pinMode(DE_RE_Pin, OUTPUT);
  digitalWrite(DE_RE_Pin, LOW); 
  Serial1.begin(9600);
  
  // 初始化 74HC595 引脚
  pinMode(PIN_595_DATA, OUTPUT);
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_595_CLOCK, OUTPUT);

  // 初始清空输出 (停止电机)
  stopMotor1();
  stopMotor2();
  updateShiftRegister(); // 确保输出为 0
  
  Serial.println("System Ready");

  // 初始化时间戳
  unsigned long currentTime = millis();
  motor1LastStepTime = currentTime;
  motor2LastStepTime = currentTime;
}

void loop() {
  handleStepperMotor(millis());
  handleRS485Commands();
}

// ================= 核心控制逻辑 =================

// 将数据发送到 74HC595
void updateShiftRegister() {
  // 拼接数据：高4位是电机2，低4位是电机1
  currentShiftOutput = (motor2Nibble << 4) | (motor1Nibble & 0x0F);

  digitalWrite(PIN_595_LATCH, LOW); // 拉低锁存，准备传输
  // LSBFIRST: 低位先出。QA 是数据的最低位(Bit 0)，QH 是最高位(Bit 7)
  // 我们的连接是 QA->电机1，QH->电机2，所以刚好对应
  shiftOut(PIN_595_DATA, PIN_595_CLOCK, LSBFIRST, currentShiftOutput);
  digitalWrite(PIN_595_LATCH, HIGH); // 拉高锁存，输出数据
}

void handleStepperMotor(unsigned long currentTime) {
  // --- 电机 1 逻辑 ---
  if(!motor1Running && motor1Enabled) {
    startMotor1();
  }

  if(motor1Running && (currentTime - motor1LastStepTime >= STEP_DELAY)) { 
    calculatePhaseMotor1(); // 计算电机1的新状态
    motor1StepPhase++;

    if(motor1StepPhase >= 4) {
      motor1StepPhase = 0;
      motor1StepCycle++;
      if(motor1StepCycle >= motor1TargetSteps) {
        finishRevolutionMotor1();
      }
    }
    motor1LastStepTime = currentTime;
    updateShiftRegister(); // 更新硬件输出
  }

  // --- 电机 2 逻辑 ---
  if(!motor2Running && motor2Enabled) {
    startMotor2();
  }

  if(motor2Running && (currentTime - motor2LastStepTime >= STEP_DELAY)) { 
    calculatePhaseMotor2(); // 计算电机2的新状态
    motor2StepPhase++;

    if(motor2StepPhase >= 4) {
      motor2StepPhase = 0;
      motor2StepCycle++;
      if(motor2StepCycle >= motor2TargetSteps) {
        finishRevolutionMotor2();
      }
    }
    motor2LastStepTime = currentTime;
    updateShiftRegister(); // 更新硬件输出
  }
}

// 计算电机1的相位数据 (低4位)
void calculatePhaseMotor1() {
  if (motor1Direction) {
    motor1Nibble = stepPatternHex[motor1StepPhase];
  } else {
    motor1Nibble = stepPatternHexRev[motor1StepPhase];
  }
}

// 计算电机2的相位数据 (高4位)
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
  Serial.println("Start Motor 1");
}

void startMotor2() {
  if(motor2Running) return; 
  motor2RevolutionCount++;
  motor2StepCycle = 0;
  motor2StepPhase = 0;
  motor2TargetSteps = STEPS_PER_REVOLUTION;
  motor2Running = true;
  Serial.println("Start Motor 2");
}

void finishRevolutionMotor1() {
  motor1Running = false;
  stopMotor1(); // 这会把 motor1Nibble 设为 0
  motor1Direction = !motor1Direction;
  updateShiftRegister(); // 立即更新以停止输出
}

void finishRevolutionMotor2() {
  motor2Running = false;
  stopMotor2(); // 这会把 motor2Nibble 设为 0
  motor2Direction = !motor2Direction;
  updateShiftRegister(); // 立即更新以停止输出
}

void stopMotor1() {
  motor1Nibble = 0x00; // 所有相位关闭
}

void stopMotor2() {
  motor2Nibble = 0x00; // 所有相位关闭
}

// ================= RS485 通信 (保持不变) =================

void sendHex485(byte data[8]) {
  digitalWrite(DE_RE_Pin, HIGH);
  delayMicroseconds(20);
  Serial1.write(data, 8);
  Serial1.flush();
  delayMicroseconds(20);
  digitalWrite(DE_RE_Pin, LOW);

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

  if (memcmp(cmd, motor1OnCmd, 8) == 0) {
    motor1Enabled = true;
    Serial.println("CMD: Enable M1");
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor1OffCmd, 8) == 0) {
    motor1Enabled = false;
    motor1Running = false;
    stopMotor1();
    updateShiftRegister(); // 立即生效
    Serial.println("CMD: Stop M1");
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2OnCmd, 8) == 0) {
    motor2Enabled = true;
    Serial.println("CMD: Enable M2");
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2OffCmd, 8) == 0) {
    motor2Enabled = false;
    motor2Running = false;
    stopMotor2();
    updateShiftRegister(); // 立即生效
    Serial.println("CMD: Stop M2");
    sendHex485(cmd);
  }
}