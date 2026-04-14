/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 夾爪控制測試
 *  子控板：ESP32-C3 Super Mini
 * =====================================================
 *
 *  功能：
 *  - 透過 ESP-NOW 接收主控板（ESP32）指令
 *  - 控制夾爪的 r（手臂伸縮）與 θ（手臂旋轉角度）
 *  - 使用 L298N 驅動 XD-25GA 370 直流減速馬達
 *  - 限位開關 (Limit Switch) 安全保護
 *  - Serial Monitor 手動測試模式
 *
 *  座標系統（圓柱座標）：
 *  - r  ：手臂伸縮（線性滑軌，由馬達 A 驅動）
 *  - θ  ：手臂旋轉角度（由馬達 B 驅動）
 *
 *  指令協議（ESP-NOW 接收 / Serial 手動測試）：
 *    'w' = 手臂伸出 (r+)
 *    's' = 手臂縮回 (r-)
 *    'a' = 手臂左轉 (θ-)
 *    'd' = 手臂右轉 (θ+)
 *    '0' = 全部停止
 *    '+' = 加速
 *    '-' = 減速
 *
 *  硬體接線：
 *  ESP32-C3 Super Mini   ->   L298N
 *  ─────────────────────────────────
 *  GPIO 0  -> IN1 (馬達A: 手臂伸縮 r)
 *  GPIO 1  -> IN2
 *  GPIO 3  -> ENA (PWM)
 *  GPIO 4  -> IN3 (馬達B: 手臂旋轉 θ)
 *  GPIO 5  -> IN4
 *  GPIO 6  -> ENB (PWM)
 *  GND     -> GND (共地)
 *
 *  限位開關：
 *  GPIO 7  -> r 伸出極限 (Limit Switch, 常開 NO, 接 GND)
 *  GPIO 8  -> r 縮回極限
 *  GPIO 9  -> θ 左極限
 *  GPIO 10 -> θ 右極限
 *
 *  注意：
 *  - ESP32-C3 不支援 BluetoothSerial，改用 ESP-NOW
 *  - L298N ENA/ENB 跳線帽要拔掉才能用 PWM 調速
 *  - 370 減速箱有自鎖效應，斷電後可保持位置
 */

#include <esp_now.h>
#include <WiFi.h>

// =====================================================
//  ESP-NOW 資料格式（與主控板 MainController 一致）
// =====================================================
// 夾爪指令列舉
enum GripperCommand {
  CMD_GRIPPER_STOP = 0,
  CMD_ARM_EXTEND,    // r+ 手臂伸出
  CMD_ARM_RETRACT,   // r- 手臂縮回
  CMD_ARM_LEFT,      // θ- 手臂左轉
  CMD_ARM_RIGHT,     // θ+ 手臂右轉
};

// ESP-NOW 接收資料格式
typedef struct gripper_now_message {
  uint8_t command;
  uint8_t speed;
} gripper_now_message;

gripper_now_message incomingMsg;
bool newDataReceived = false;

// =====================================================
//  L298N 腳位定義 — ESP32-C3 Super Mini
// =====================================================
// 馬達 A：手臂伸縮 (r 方向)
const int motorR_Pin1 = 0;
const int motorR_Pin2 = 1;
const int motorR_EN   = 3;   // PWM 調速

// 馬達 B：手臂旋轉 (θ 方向)
const int motorT_Pin1 = 4;
const int motorT_Pin2 = 5;
const int motorT_EN   = 6;   // PWM 調速

// =====================================================
//  限位開關腳位
//  接法：GPIO ---- Limit Switch (NO) ---- GND
//  使用 INPUT_PULLUP：未觸發 = HIGH，觸發 = LOW
// =====================================================
const int limitR_Extend  = 7;   // r 伸出極限
const int limitR_Retract = 8;   // r 縮回極限
const int limitT_Left    = 9;   // θ 左極限
const int limitT_Right   = 10;  // θ 右極限

// =====================================================
//  PWM 設定
// =====================================================
const int pwmFreq       = 30000;
const int pwmResolution = 8;      // 8-bit -> 0~255
const int pwmChannel_R  = 0;      // r 馬達 PWM 通道
const int pwmChannel_T  = 1;      // θ 馬達 PWM 通道

// =====================================================
//  速度設定
// =====================================================
int dutyCycle = 180;              // 預設速度 (0~255)
const int SPEED_MIN  = 80;       // 最低可動速度
const int SPEED_MAX  = 255;      // 最高速度
const int SPEED_STEP = 20;       // 加減速步進

// =====================================================
//  安全保護：最大通電時間 (ms)
//  超過此時間自動斷電，防止馬達卡死燒毀
// =====================================================
const unsigned long MAX_RUN_TIME = 5000;  // 5 秒
unsigned long motorR_startTime = 0;
unsigned long motorT_startTime = 0;
bool motorR_running = false;
bool motorT_running = false;

// =====================================================
//  ESP-NOW 回調：接收到資料時觸發
// =====================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(gripper_now_message)) {
    memcpy(&incomingMsg, data, sizeof(gripper_now_message));
    newDataReceived = true;

    // Debug 輸出
    Serial.print("ESP-NOW Recv -> CMD: ");
    Serial.print(incomingMsg.command);
    Serial.print(", Speed: ");
    Serial.println(incomingMsg.speed);
  }
}

// =====================================================
//  馬達控制函式
// =====================================================

// --- r 方向：手臂伸出 ---
void armExtend() {
  // 檢查限位開關
  if (digitalRead(limitR_Extend) == LOW) {
    Serial.println("!! r Extend Limit Hit -> STOP");
    armR_Stop();
    return;
  }
  digitalWrite(motorR_Pin1, HIGH);
  digitalWrite(motorR_Pin2, LOW);
  ledcWrite(motorR_EN, dutyCycle);

  if (!motorR_running) {
    motorR_running = true;
    motorR_startTime = millis();
  }
  Serial.print("Arm Extend (r+), speed=");
  Serial.println(dutyCycle);
}

// --- r 方向：手臂縮回 ---
void armRetract() {
  // 檢查限位開關
  if (digitalRead(limitR_Retract) == LOW) {
    Serial.println("!! r Retract Limit Hit -> STOP");
    armR_Stop();
    return;
  }
  digitalWrite(motorR_Pin1, LOW);
  digitalWrite(motorR_Pin2, HIGH);
  ledcWrite(motorR_EN, dutyCycle);

  if (!motorR_running) {
    motorR_running = true;
    motorR_startTime = millis();
  }
  Serial.print("Arm Retract (r-), speed=");
  Serial.println(dutyCycle);
}

// --- r 馬達停止 ---
void armR_Stop() {
  digitalWrite(motorR_Pin1, LOW);
  digitalWrite(motorR_Pin2, LOW);
  ledcWrite(motorR_EN, 0);
  motorR_running = false;
  Serial.println("Motor R STOP");
}

// --- θ 方向：手臂左轉 ---
void armLeft() {
  // 檢查限位開關
  if (digitalRead(limitT_Left) == LOW) {
    Serial.println("!! Theta Left Limit Hit -> STOP");
    armT_Stop();
    return;
  }
  digitalWrite(motorT_Pin1, HIGH);
  digitalWrite(motorT_Pin2, LOW);
  ledcWrite(motorT_EN, dutyCycle);

  if (!motorT_running) {
    motorT_running = true;
    motorT_startTime = millis();
  }
  Serial.print("Arm Left (theta-), speed=");
  Serial.println(dutyCycle);
}

// --- θ 方向：手臂右轉 ---
void armRight() {
  // 檢查限位開關
  if (digitalRead(limitT_Right) == LOW) {
    Serial.println("!! Theta Right Limit Hit -> STOP");
    armT_Stop();
    return;
  }
  digitalWrite(motorT_Pin1, LOW);
  digitalWrite(motorT_Pin2, HIGH);
  ledcWrite(motorT_EN, dutyCycle);

  if (!motorT_running) {
    motorT_running = true;
    motorT_startTime = millis();
  }
  Serial.print("Arm Right (theta+), speed=");
  Serial.println(dutyCycle);
}

// --- θ 馬達停止 ---
void armT_Stop() {
  digitalWrite(motorT_Pin1, LOW);
  digitalWrite(motorT_Pin2, LOW);
  ledcWrite(motorT_EN, 0);
  motorT_running = false;
  Serial.println("Motor T STOP");
}

// --- 全部停止 ---
void allStop() {
  armR_Stop();
  armT_Stop();
  Serial.println("=== ALL STOP ===");
}

// =====================================================
//  安全保護：超時自動斷電
// =====================================================
void checkTimeout() {
  if (motorR_running && (millis() - motorR_startTime > MAX_RUN_TIME)) {
    Serial.println("!! Motor R TIMEOUT -> Safety STOP");
    armR_Stop();
  }
  if (motorT_running && (millis() - motorT_startTime > MAX_RUN_TIME)) {
    Serial.println("!! Motor T TIMEOUT -> Safety STOP");
    armT_Stop();
  }
}

// =====================================================
//  安全保護：即時限位開關檢測
// =====================================================
void checkLimitSwitches() {
  // r 方向限位
  if (motorR_running) {
    // 正在伸出時，檢查伸出極限
    if (digitalRead(motorR_Pin1) == HIGH && digitalRead(limitR_Extend) == LOW) {
      Serial.println("!! r Extend Limit -> Emergency STOP");
      armR_Stop();
    }
    // 正在縮回時，檢查縮回極限
    if (digitalRead(motorR_Pin2) == HIGH && digitalRead(limitR_Retract) == LOW) {
      Serial.println("!! r Retract Limit -> Emergency STOP");
      armR_Stop();
    }
  }

  // θ 方向限位
  if (motorT_running) {
    // 正在左轉時，檢查左極限
    if (digitalRead(motorT_Pin1) == HIGH && digitalRead(limitT_Left) == LOW) {
      Serial.println("!! Theta Left Limit -> Emergency STOP");
      armT_Stop();
    }
    // 正在右轉時，檢查右極限
    if (digitalRead(motorT_Pin2) == HIGH && digitalRead(limitT_Right) == LOW) {
      Serial.println("!! Theta Right Limit -> Emergency STOP");
      armT_Stop();
    }
  }
}

// =====================================================
//  處理指令（ESP-NOW 或 Serial 共用）
// =====================================================
void executeCommand(uint8_t cmd, uint8_t spd) {
  // 如果 ESP-NOW 帶了速度值，就使用它
  if (spd > 0) {
    dutyCycle = constrain(spd, SPEED_MIN, SPEED_MAX);
  }

  switch (cmd) {
    case CMD_GRIPPER_STOP:
      allStop();
      break;
    case CMD_ARM_EXTEND:
      armExtend();
      break;
    case CMD_ARM_RETRACT:
      armRetract();
      break;
    case CMD_ARM_LEFT:
      armLeft();
      break;
    case CMD_ARM_RIGHT:
      armRight();
      break;
    default:
      Serial.print("Unknown CMD: ");
      Serial.println(cmd);
      break;
  }
}

// =====================================================
//  Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);  // 等 Serial 穩定

  Serial.println("========================================");
  Serial.println("  ESP32-C3 Gripper Controller (r, theta)");
  Serial.println("========================================");

  // ----- 馬達腳位設定 -----
  pinMode(motorR_Pin1, OUTPUT);
  pinMode(motorR_Pin2, OUTPUT);
  pinMode(motorR_EN,   OUTPUT);

  pinMode(motorT_Pin1, OUTPUT);
  pinMode(motorT_Pin2, OUTPUT);
  pinMode(motorT_EN,   OUTPUT);

  // ----- 限位開關設定（內部上拉） -----
  pinMode(limitR_Extend,  INPUT_PULLUP);
  pinMode(limitR_Retract, INPUT_PULLUP);
  pinMode(limitT_Left,    INPUT_PULLUP);
  pinMode(limitT_Right,   INPUT_PULLUP);

  // ----- ESP32 PWM (LEDC) 設定 -----
  ledcAttachChannel(motorR_EN, pwmFreq, pwmResolution, pwmChannel_R);
  ledcAttachChannel(motorT_EN, pwmFreq, pwmResolution, pwmChannel_T);

  // ----- 初始停止 -----
  allStop();

  // ----- ESP-NOW 初始化 -----
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("!! ESP-NOW Init FAILED");
    // 繼續執行，仍可透過 Serial 測試
  } else {
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW Initialized, waiting for data...");
  }

  // ----- 印出使用說明 -----
  Serial.println();
  Serial.println("--- Serial Test Commands ---");
  Serial.println("  w = Arm Extend  (r+)");
  Serial.println("  s = Arm Retract (r-)");
  Serial.println("  a = Arm Left   (theta-)");
  Serial.println("  d = Arm Right  (theta+)");
  Serial.println("  0 = ALL STOP");
  Serial.println("  + = Speed UP");
  Serial.println("  - = Speed DOWN");
  Serial.println("  ? = Show status");
  Serial.println("----------------------------");
  Serial.print("Current speed: ");
  Serial.println(dutyCycle);
  Serial.println();
}

// =====================================================
//  Main Loop
// =====================================================
void loop() {
  // ----- 安全保護 -----
  checkTimeout();
  checkLimitSwitches();

  // ----- 處理 ESP-NOW 接收的指令 -----
  if (newDataReceived) {
    newDataReceived = false;
    executeCommand(incomingMsg.command, incomingMsg.speed);
  }

  // ----- Serial 手動測試模式 -----
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'w':
        executeCommand(CMD_ARM_EXTEND, 0);
        break;
      case 's':
        executeCommand(CMD_ARM_RETRACT, 0);
        break;
      case 'a':
        executeCommand(CMD_ARM_LEFT, 0);
        break;
      case 'd':
        executeCommand(CMD_ARM_RIGHT, 0);
        break;
      case '0':
        executeCommand(CMD_GRIPPER_STOP, 0);
        break;
      case '+':
        dutyCycle = constrain(dutyCycle + SPEED_STEP, SPEED_MIN, SPEED_MAX);
        Serial.print("Speed UP -> ");
        Serial.println(dutyCycle);
        break;
      case '-':
        dutyCycle = constrain(dutyCycle - SPEED_STEP, SPEED_MIN, SPEED_MAX);
        Serial.print("Speed DOWN -> ");
        Serial.println(dutyCycle);
        break;
      case '?':
        // 印出目前狀態
        Serial.println("--- Status ---");
        Serial.print("  Speed: ");
        Serial.println(dutyCycle);
        Serial.print("  Motor R running: ");
        Serial.println(motorR_running ? "YES" : "NO");
        Serial.print("  Motor T running: ");
        Serial.println(motorT_running ? "YES" : "NO");
        Serial.print("  Limit R_Ext: ");
        Serial.println(digitalRead(limitR_Extend) == LOW ? "TRIGGERED" : "OK");
        Serial.print("  Limit R_Ret: ");
        Serial.println(digitalRead(limitR_Retract) == LOW ? "TRIGGERED" : "OK");
        Serial.print("  Limit T_Lft: ");
        Serial.println(digitalRead(limitT_Left) == LOW ? "TRIGGERED" : "OK");
        Serial.print("  Limit T_Rgt: ");
        Serial.println(digitalRead(limitT_Right) == LOW ? "TRIGGERED" : "OK");
        Serial.println("--------------");
        break;
      default:
        // 忽略換行等字元
        if (c != '\n' && c != '\r') {
          Serial.print("Unknown key: ");
          Serial.println(c);
        }
        break;
    }
  }

  delay(10);  // 降低 CPU 負載
}
