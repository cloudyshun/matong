/*
 * 项目名称: STM32F103C8T6 智能马桶全功能控制系统
 * 版本: V2.0 (集成过零检测开关)
 * 
 * 硬件连接说明:
 * 1. 强电控制 (低电平有效，通过 MOC3023 驱动):
 *    - 风扇加热丝: PA1
 *    - 水泵:       PA2
 *    - 水即热装置: PA3
 *    - 粉碎泵:     PA4
 *    - [关键] 过零信号输入: PA0 (连接 LTV-814S 光耦输出)
 * 
 * 2. 步进电机 (通过 74HC595 + ULN2803):
 *    - 595 Data (SER):   PA5
 *    - 595 Latch (RCLK): PA6
 *    - 595 Clock (SRCLK): PA7
 *    - 电机1: 对应 595 输出的低4位 (Q0-Q3)
 *    - 电机2: 对应 595 输出的高4位 (Q4-Q7)
 * 
 * 3. RS485 通信:
 *    - 控制脚 (DE/RE): PA8
 *    - TX/RX: Serial1 (PA9/PA10)
 */

#include <Arduino.h>

// ================= 1. 引脚定义 =================
// 强电与过零检测
#define PIN_ZERO_CROSS PA0  // 过零中断输入
#define PIN_FAN        PA1  // 风扇加热丝
#define PIN_PUMP       PA2  // 水泵
#define PIN_HEATER     PA3  // 水即热装置
#define PIN_MACERATOR  PA4  // 粉碎泵

// 步进电机 (74HC595)
#define PIN_595_DATA   PA5
#define PIN_595_LATCH  PA6
#define PIN_595_CLOCK  PA7

// RS485
#define DE_RE_Pin      PA8

// ================= 2. 参数设置 =================
#define STEPS_PER_REVOLUTION 500  // 电机转一圈的步数 (根据实际电机调整)
#define STEP_DELAY 5              // 步进速度 (毫秒/步)

// ================= 3. 全局变量 =================

// --- [核心] 强电负载的目标状态 (volatile 必不可少，因为在中断中读取) ---
// true = 开启(输出低电平), false = 关闭(输出高电平)
volatile bool targetStateFan = false;
volatile bool targetStatePump = false;
volatile bool targetStateHeater = false;
volatile bool targetStateMacerator = false;

// --- 步进电机相关变量 ---
byte currentShiftOutput = 0; // 595当前的输出字节
byte motor1Nibble = 0;       // 电机1的4位状态
byte motor2Nibble = 0;       // 电机2的4位状态

int nextTargetSteps1 = STEPS_PER_REVOLUTION; 
int nextTargetSteps2 = STEPS_PER_REVOLUTION; 

// 复合动作指令的状态机
int command3Status = 0; 

// 电机1 运行时参数
bool motor1Running = false;
int motor1StepPhase = 0;      // 当前相位 (0-3)
int motor1StepCycle = 0;      // 当前步数计数
int motor1TargetSteps = 0;    // 目标总步数
unsigned long motor1LastStepTime = 0;
bool motor1Direction = true;  // true=正转, false=反转
bool motor1Enabled = false;   
int motor1RevolutionCount = 0;

// 电机2 运行时参数
bool motor2Running = false;
int motor2StepPhase = 0;
int motor2StepCycle = 0;
int motor2TargetSteps = 0;
unsigned long motor2LastStepTime = 0;
bool motor2Direction = true;  
bool motor2Enabled = false;   
int motor2RevolutionCount = 0;

// RS485 接收缓存
byte rs485Buffer[8];
int rs485Index = 0;

// 双相励磁序列表 (两相通电，扭矩大)
// 对应二进制: 1100, 0110, 0011, 1001
const byte stepPatternHex[4] = {0x0C, 0x06, 0x03, 0x09};
const byte stepPatternHexRev[4] = {0x09, 0x03, 0x06, 0x0C};

// ================= 4. 指令集定义 (Hex) =================
// 步进电机指令 (1-4)
const byte motor1FwdCmd[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00}; // M1 正转
const byte motor1RevCmd[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00}; // M1 反转
const byte motor2FwdCmd[8] = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00}; // M2 正转
const byte motor2RevCmd[8] = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00}; // M2 反转
const byte cmdAction1[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00}; // M1逆 M2正
const byte cmdAction2[8]   = {0xEE, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00}; // 全逆
const byte cmdAction3[8]   = {0xEE, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00}; // 复合动作

// 强电负载指令 (5-12)
const byte cmdFanOn[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00}; // 风扇开
const byte cmdFanOff[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01}; // 风扇关
const byte cmdPumpOn[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x00}; // 水泵开
const byte cmdPumpOff[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x02, 0x01}; // 水泵关
const byte cmdHeatOn[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x03, 0x00}; // 加热开
const byte cmdHeatOff[8] = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x03, 0x01}; // 加热关
const byte cmdMacOn[8]   = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x04, 0x00}; // 粉碎开
const byte cmdMacOff[8]  = {0xEE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x04, 0x01}; // 粉碎关


// ================= 5. 函数声明 =================
void zeroCrossingISR(); // 中断服务函数
void updateShiftRegister();
void stopMotor1(); void stopMotor2();
void startMotor1(); void startMotor2();
void calculatePhaseMotor1(); void calculatePhaseMotor2();
void finishRevolutionMotor1(); void finishRevolutionMotor2();
void handleStepperMotor(unsigned long currentTime);
void handleRS485Commands();
void processHexCommand(byte cmd[8]);
void sendHex485(byte data[8]);

// ================= 6. Setup 初始化 =================
void setup() {
  // 调试串口 (USB)
  Serial.begin(9600);
  delay(1000);
  Serial.println("System Initializing...");

  // RS485 初始化
  pinMode(DE_RE_Pin, OUTPUT);
  digitalWrite(DE_RE_Pin, LOW); // 默认接收模式
  Serial1.begin(9600);          // PA9/PA10
  
  // 步进电机引脚
  pinMode(PIN_595_DATA, OUTPUT);
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_595_CLOCK, OUTPUT);

  // --- 强电负载初始化 ---
  // 上电默认关闭 (高电平)
  pinMode(PIN_FAN, OUTPUT); 
  digitalWrite(PIN_FAN, HIGH);
  
  pinMode(PIN_PUMP, OUTPUT); 
  digitalWrite(PIN_PUMP, HIGH);
  
  pinMode(PIN_HEATER, OUTPUT); 
  digitalWrite(PIN_HEATER, HIGH);
  
  pinMode(PIN_MACERATOR, OUTPUT); 
  digitalWrite(PIN_MACERATOR, HIGH);

  // --- [关键] 开启 PA0 过零检测中断 ---
  // PA0 连接光耦，平时高电平，过零时低电平。
  // 检测下降沿 (FALLING) 代表进入过零点。
  pinMode(PIN_ZERO_CROSS, INPUT); 
  attachInterrupt(digitalPinToInterrupt(PIN_ZERO_CROSS), zeroCrossingISR, FALLING);

  // 步进电机初始归零
  stopMotor1();
  stopMotor2();
  updateShiftRegister(); 
  
  // 默认启动参数
  nextTargetSteps1 = STEPS_PER_REVOLUTION;
  nextTargetSteps2 = STEPS_PER_REVOLUTION;
  motor1Direction = true; 
  motor2Direction = true; 
  motor1Enabled = true;
  motor2Enabled = true;
  command3Status = 0; 
  
  Serial.println("System Ready: Zero-Crossing Enabled");
}

// ================= 7. 主循环 =================
void loop() {
  // 步进电机是非阻塞的，需要不断调用
  handleStepperMotor(millis());
  
  // 检查 RS485 指令
  handleRS485Commands();
}

// ================= 8. 中断服务函数 (ISR) =================
// 只要交流电有电，该函数每 10ms (50Hz) 会自动执行一次
// 作用：将 targetState 的状态应用到 IO 口，实现过零开关
void zeroCrossingISR() {
  // 写入物理引脚 (低电平有效)
  digitalWrite(PIN_FAN,       targetStateFan       ? LOW : HIGH);
  digitalWrite(PIN_PUMP,      targetStatePump      ? LOW : HIGH);
  digitalWrite(PIN_HEATER,    targetStateHeater    ? LOW : HIGH);
  digitalWrite(PIN_MACERATOR, targetStateMacerator ? LOW : HIGH);
}

// ================= 9. RS485 通信处理 =================

void handleRS485Commands() {
  while (Serial1.available()) {
    byte receivedByte = Serial1.read();
    // 简单的帧头检测 (0xEE)
    if (receivedByte == 0xEE) {
      rs485Buffer[0] = receivedByte;
      rs485Index = 1;
      unsigned long startTime = millis();
      // 读取剩余7字节，带超时
      while (rs485Index < 8) {
        if (Serial1.available()) {
          rs485Buffer[rs485Index++] = Serial1.read();
        }
        if (millis() - startTime > 20) { // 20ms 超时
          rs485Index = 0; 
          return; 
        }
      }
      // 收到完整8字节，处理
      if (rs485Index == 8) {
        processHexCommand(rs485Buffer);
      }
      rs485Index = 0;
    }
  }
}

void sendHex485(byte data[8]) {
  digitalWrite(DE_RE_Pin, HIGH); // 发送模式
  delayMicroseconds(20);         // 等待电平稳定
  Serial1.write(data, 8);
  Serial1.flush();               // 等待发送完成
  delayMicroseconds(20);
  digitalWrite(DE_RE_Pin, LOW);  // 切换回接收模式
  Serial.println("Ack Sent");
}

void processHexCommand(byte cmd[8]) {
  if (cmd[0] != 0xEE) return;
  
  // 重置电机默认目标
  nextTargetSteps1 = STEPS_PER_REVOLUTION; 
  nextTargetSteps2 = STEPS_PER_REVOLUTION; 
  // 注意：command3Status 在收到新指令时需要根据情况重置，但在电机未跑完时不建议打断

  // --- A. 强电控制指令 (只修改变量，不操作IO) ---
  
  // 1. 风扇控制
  if (memcmp(cmd, cmdFanOn, 8) == 0) {
    targetStateFan = true;
    Serial.println("CMD: Fan ON (Wait ZC)");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdFanOff, 8) == 0) {
    targetStateFan = false;
    Serial.println("CMD: Fan OFF (Wait ZC)");
    sendHex485(cmd); return;
  }
  
  // 2. 水泵控制
  else if (memcmp(cmd, cmdPumpOn, 8) == 0) {
    targetStatePump = true;
    Serial.println("CMD: Pump ON (Wait ZC)");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdPumpOff, 8) == 0) {
    targetStatePump = false;
    Serial.println("CMD: Pump OFF (Wait ZC)");
    sendHex485(cmd); return;
  }
  
  // 3. 水即热控制
  else if (memcmp(cmd, cmdHeatOn, 8) == 0) {
    targetStateHeater = true;
    Serial.println("CMD: Heater ON (Wait ZC)");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdHeatOff, 8) == 0) {
    targetStateHeater = false;
    Serial.println("CMD: Heater OFF (Wait ZC)");
    sendHex485(cmd); return;
  }
  
  // 4. 粉碎泵控制
  else if (memcmp(cmd, cmdMacOn, 8) == 0) {
    targetStateMacerator = true;
    Serial.println("CMD: Macerator ON (Wait ZC)");
    sendHex485(cmd); return;
  }
  else if (memcmp(cmd, cmdMacOff, 8) == 0) {
    targetStateMacerator = false;
    Serial.println("CMD: Macerator OFF (Wait ZC)");
    sendHex485(cmd); return;
  }

  // --- B. 步进电机控制指令 ---
  
  // 收到新电机指令，先重置电机状态机
  command3Status = 0; 
  
  if (memcmp(cmd, cmdAction1, 8) == 0) {
    motor1Running = false; motor2Running = false;
    motor1Direction = false; // M1 逆
    motor2Direction = true;  // M2 正
    motor1Enabled = true; motor2Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, cmdAction2, 8) == 0) {
    motor1Running = false; motor2Running = false;
    motor1Direction = false; // M1 逆
    motor2Direction = false; // M2 逆
    motor1Enabled = true; motor2Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, cmdAction3, 8) == 0) {
    motor1Running = false; motor2Running = false;
    command3Status = 1;      // 激活复合动作
    motor1Direction = false; // M1 逆
    motor2Direction = false; // M2 逆
    nextTargetSteps1 = 500;  // 1圈
    nextTargetSteps2 = 500;  // 1圈
    motor1Enabled = true; motor2Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor1FwdCmd, 8) == 0) {
    motor1Running = false; 
    motor1Direction = true; 
    motor1Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor1RevCmd, 8) == 0) {
    motor1Running = false; 
    motor1Direction = false; 
    motor1Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2FwdCmd, 8) == 0) {
    motor2Running = false; 
    motor2Direction = true; 
    motor2Enabled = true;
    sendHex485(cmd);
  }
  else if (memcmp(cmd, motor2RevCmd, 8) == 0) {
    motor2Running = false; 
    motor2Direction = false; 
    motor2Enabled = true;
    sendHex485(cmd);
  }
}

// ================= 10. 步进电机逻辑 =================

// 将缓存数据写入 74HC595
void updateShiftRegister() {
  currentShiftOutput = (motor2Nibble << 4) | (motor1Nibble & 0x0F);
  digitalWrite(PIN_595_LATCH, LOW); 
  shiftOut(PIN_595_DATA, PIN_595_CLOCK, LSBFIRST, currentShiftOutput);
  digitalWrite(PIN_595_LATCH, HIGH); 
}

// 电机主处理函数 (在 loop 中调用)
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
      if(motor2StepCycle >= motor2TargetSteps) {
        finishRevolutionMotor2();
      }
    }
    motor2LastStepTime = currentTime;
    updateShiftRegister(); 
  }
}

// 计算电机1的励磁状态
void calculatePhaseMotor1() {
  if (motor1Direction) motor1Nibble = stepPatternHex[motor1StepPhase];
  else motor1Nibble = stepPatternHexRev[motor1StepPhase];
}

// 计算电机2的励磁状态
void calculatePhaseMotor2() {
  if (motor2Direction) motor2Nibble = stepPatternHex[motor2StepPhase];
  else motor2Nibble = stepPatternHexRev[motor2StepPhase];
}

// 启动电机1
void startMotor1() {
  if(motor1Running) return; 
  motor1RevolutionCount++;
  motor1StepCycle = 0;
  motor1StepPhase = 0;
  motor1TargetSteps = nextTargetSteps1; 
  motor1Running = true;
}

// 启动电机2
void startMotor2() {
  if(motor2Running) return; 
  motor2RevolutionCount++;
  motor2StepCycle = 0;
  motor2StepPhase = 0;
  motor2TargetSteps = nextTargetSteps2; 
  motor2Running = true;
}

// 电机1 停止逻辑
void finishRevolutionMotor1() {
  motor1Running = false;
  motor1Enabled = false; 
  stopMotor1();          
  updateShiftRegister(); 
}

// 电机2 停止逻辑 (包含复杂动作 Command3)
void finishRevolutionMotor2() {
  motor2Running = false;
  
  // Command 3 特殊阶段处理
  if (command3Status == 1) {
    command3Status = 2;     // 进入阶段二
    motor2Direction = true; // 变正转
    nextTargetSteps2 = 200; // 0.4圈
    startMotor2();          // 立即重启
    Serial.println("CMD3: Phase 2 Start");
    updateShiftRegister();
    return; // 退出，保持运行
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