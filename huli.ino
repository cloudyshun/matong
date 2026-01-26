/*
 * STM32F103C8T6 纯步进电机控制系统 + RS485通信
 * 
 * 硬件连接 (基于提供的原理图):
 * ---------------------------------------------------------
 * [电机 1] (原理图右侧 U55)
 * - A相 -> PB0
 * - B相 -> PB1
 * - C相 -> PB10
 * - D相 -> PB11
 * 
 * [电机 2] (原理图左侧 U22)
 * - A相 -> PA3
 * - B相 -> PA4
 * - C相 -> PA5
 * - D相 -> PA6
 * ---------------------------------------------------------
 * - RS485 DE/RE -> PB9
 * - RS485 TX/RX -> PA9/PA10 (Serial1)
 */

#include <Arduino.h>

// RS485控制引脚
#define DE_RE_Pin PB9

// ================= 引脚定义 (严格对应原理图) =================

// 【电机 1】定义 (原理图右侧，PB口)
#define M1_PIN_A  PB0   // U55 Pin 1 -> NET7
#define M1_PIN_B  PB1   // U55 Pin 2 -> NET8
#define M1_PIN_C  PB10  // U55 Pin 3 -> NET9
#define M1_PIN_D  PB11  // U55 Pin 4 -> NET10

// 【电机 2】定义 (原理图左侧，PA口)
#define M2_PIN_A  PA3   // U22 Pin 1 -> NET3
#define M2_PIN_B  PA4   // U22 Pin 2 -> NET4
#define M2_PIN_C  PA5   // U22 Pin 3 -> NET5
#define M2_PIN_D  PA6   // U22 Pin 4 -> NET6

// 步进电机参数设置
#define STEPS_PER_REVOLUTION 500  // 每圈步数
#define STEP_DELAY 5             // 步进周期延时(毫秒)速度

// ================= 全局变量 =================

// 电机1 状态变量 (控制 PB0-PB11)
bool motor1Running = false;
int motor1StepPhase = 0;
int motor1StepCycle = 0;
int motor1TargetSteps = 0;
unsigned long motor1LastStepTime = 0;
bool motor1Direction = true;  // true=正向
bool motor1Enabled = false;   // RS485开启/关闭标志
int motor1RevolutionCount = 0;

// 电机2 状态变量 (控制 PA3-PA6)
bool motor2Running = false;
int motor2StepPhase = 0;
int motor2StepCycle = 0;
int motor2TargetSteps = 0;
unsigned long motor2LastStepTime = 0;
bool motor2Direction = true;  // true=正向
bool motor2Enabled = false;   // RS485开启/关闭标志
int motor2RevolutionCount = 0;

// RS485控制指令 (HEX)
const byte motor1OnCmd[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00}; // 电机1 开启
const byte motor1OffCmd[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00}; // 电机1 关闭
const byte motor2OnCmd[8]  = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00}; // 电机2 开启
const byte motor2OffCmd[8] = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00}; // 电机2 关闭

// RS485接收缓冲区
byte rs485Buffer[8];
int rs485Index = 0;

// 双相激励序列表 (正转)
const bool stepPattern[4][4] = {
  {HIGH, HIGH, LOW,  LOW },  // AB
  {LOW,  HIGH, HIGH, LOW },  // BC
  {LOW,  LOW,  HIGH, HIGH},  // CD
  {HIGH, LOW,  LOW,  HIGH}   // DA
};

// 双相激励序列表 (反转)
const bool stepPatternReverse[4][4] = {
  {HIGH, LOW,  LOW,  HIGH},  // DA
  {LOW,  LOW,  HIGH, HIGH},  // CD
  {LOW,  HIGH, HIGH, LOW },  // BC
  {HIGH, HIGH, LOW,  LOW }   // AB
};

// 函数声明
void stopMotor1();
void stopMotor2();
void startMotor1();
void startMotor2();
void executeStepPhaseMotor1();
void executeStepPhaseMotor2();
void finishRevolutionMotor1();
void finishRevolutionMotor2();
void handleStepperMotor(unsigned long currentTime);
void handleRS485Commands();
void processHexCommand(byte cmd[8]);
void sendHex485(byte data[8]);

void setup() {
  Serial.begin(9600); // 调试串口
  delay(1000);
  Serial.println("STM32 Dual Motor Control Started...");

  // 初始化RS485
  pinMode(DE_RE_Pin, OUTPUT);
  digitalWrite(DE_RE_Pin, LOW);  // 接收模式
  Serial1.begin(9600);
  Serial.println("RS485 Ready");
  
  // 初始化【电机1】引脚 (PB口)
  pinMode(M1_PIN_A, OUTPUT);
  pinMode(M1_PIN_B, OUTPUT);
  pinMode(M1_PIN_C, OUTPUT);
  pinMode(M1_PIN_D, OUTPUT);
  
  // 初始化【电机2】引脚 (PA口)
  pinMode(M2_PIN_A, OUTPUT);
  pinMode(M2_PIN_B, OUTPUT);
  pinMode(M2_PIN_C, OUTPUT);
  pinMode(M2_PIN_D, OUTPUT);

  // 初始停止所有电机
  stopMotor1();
  stopMotor2();
  Serial.println("Motors Initialized");

  // 初始化时间戳
  unsigned long currentTime = millis();
  motor1LastStepTime = currentTime;
  motor2LastStepTime = currentTime;
}

void loop() {
  // 任务1: 步进电机逻辑
  handleStepperMotor(millis());

  // 任务2: RS485指令处理
  handleRS485Commands();
}

// 步进电机核心控制逻辑
void handleStepperMotor(unsigned long currentTime) {
  // ---------------- 电机 1 (PB口) 控制 ----------------
  if(!motor1Running && motor1Enabled) {
    startMotor1();
  }

  if(motor1Running && (currentTime - motor1LastStepTime >= STEP_DELAY)) { 
    executeStepPhaseMotor1(); // 驱动 PB 引脚
    motor1StepPhase++;

    if(motor1StepPhase >= 4) {
      motor1StepPhase = 0;
      motor1StepCycle++;
      if(motor1StepCycle >= motor1TargetSteps) {
        finishRevolutionMotor1();
      }
    }
    motor1LastStepTime = currentTime;
  }

  // ---------------- 电机 2 (PA口) 控制 ----------------
  if(!motor2Running && motor2Enabled) {
    startMotor2();
  }

  if(motor2Running && (currentTime - motor2LastStepTime >= STEP_DELAY)) { 
    executeStepPhaseMotor2(); // 驱动 PA 引脚
    motor2StepPhase++;

    if(motor2StepPhase >= 4) {
      motor2StepPhase = 0;
      motor2StepCycle++;
      if(motor2StepCycle >= motor2TargetSteps) {
        finishRevolutionMotor2();
      }
    }
    motor2LastStepTime = currentTime;
  }
}

// 执行电机1相位 (PB0, PB1, PB10, PB11)
void executeStepPhaseMotor1() {
  const bool (*pattern)[4] = motor1Direction ? stepPattern : stepPatternReverse;
  digitalWrite(M1_PIN_A, pattern[motor1StepPhase][0]);
  digitalWrite(M1_PIN_B, pattern[motor1StepPhase][1]);
  digitalWrite(M1_PIN_C, pattern[motor1StepPhase][2]);
  digitalWrite(M1_PIN_D, pattern[motor1StepPhase][3]);
}

// 执行电机2相位 (PA3, PA4, PA5, PA6)
void executeStepPhaseMotor2() {
  const bool (*pattern)[4] = motor2Direction ? stepPattern : stepPatternReverse;
  digitalWrite(M2_PIN_A, pattern[motor2StepPhase][0]);
  digitalWrite(M2_PIN_B, pattern[motor2StepPhase][1]);
  digitalWrite(M2_PIN_C, pattern[motor2StepPhase][2]);
  digitalWrite(M2_PIN_D, pattern[motor2StepPhase][3]);
}

// 启动电机1
void startMotor1() {
  if(motor1Running) return; 
  motor1RevolutionCount++;
  motor1StepCycle = 0;
  motor1StepPhase = 0;
  motor1TargetSteps = STEPS_PER_REVOLUTION;
  motor1Running = true;
  Serial.println("Start Motor 1 (PB Pins)");
}

// 启动电机2
void startMotor2() {
  if(motor2Running) return; 
  motor2RevolutionCount++;
  motor2StepCycle = 0;
  motor2StepPhase = 0;
  motor2TargetSteps = STEPS_PER_REVOLUTION;
  motor2Running = true;
  Serial.println("Start Motor 2 (PA Pins)");
}

// 完成一圈：电机1
void finishRevolutionMotor1() {
  motor1Running = false;
  stopMotor1();
  motor1Direction = !motor1Direction; // 自动反向
}

// 完成一圈：电机2
void finishRevolutionMotor2() {
  motor2Running = false;
  stopMotor2();
  motor2Direction = !motor2Direction; // 自动反向
}

// 停止电机1 (拉低PB口)
void stopMotor1() {
  digitalWrite(M1_PIN_A, LOW);
  digitalWrite(M1_PIN_B, LOW);
  digitalWrite(M1_PIN_C, LOW);
  digitalWrite(M1_PIN_D, LOW);
}

// 停止电机2 (拉低PA口)
void stopMotor2() {
  digitalWrite(M2_PIN_A, LOW);
  digitalWrite(M2_PIN_B, LOW);
  digitalWrite(M2_PIN_C, LOW);
  digitalWrite(M2_PIN_D, LOW);
}

// RS485发送
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

// RS485接收处理
void handleRS485Commands() {
  while (Serial1.available()) {
    byte receivedByte = Serial1.read();
    if (receivedByte == 0xEE) { // 帧头
      rs485Buffer[0] = receivedByte;
      rs485Index = 1;
      unsigned long startTime = millis();
      while (rs485Index < 8) {
        if (Serial1.available()) {
          rs485Buffer[rs485Index++] = Serial1.read();
        }
        if (millis() - startTime > 20) { // 超时
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

// 解析指令
void processHexCommand(byte cmd[8]) {
  if (cmd[0] != 0xEE) return;

  // 比较指令
  if (memcmp(cmd, motor1OnCmd, 8) == 0) {
    motor1Enabled = true; // 开启电机1 (PB)
    Serial.println("CMD: Enable Motor 1");
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor1OffCmd, 8) == 0) {
    motor1Enabled = false; // 关闭电机1 (PB)
    motor1Running = false;
    stopMotor1();
    Serial.println("CMD: Stop Motor 1");
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2OnCmd, 8) == 0) {
    motor2Enabled = true; // 开启电机2 (PA)
    Serial.println("CMD: Enable Motor 2");
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2OffCmd, 8) == 0) {
    motor2Enabled = false; // 关闭电机2 (PA)
    motor2Running = false;
    stopMotor2();
    Serial.println("CMD: Stop Motor 2");
    sendHex485(cmd);
  }
}