/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 Servo 子控板
 *  ESP32-C3 #3 ServoClawController
 * =====================================================
 *
 *  功能：
 *  - 透過 ESP-NOW 接收主控板（ESP32）Servo / 夾爪指令
 *  - 控制 r 齒條伸縮（MG996R 360° 連續旋轉舵機）
 *  - 控制 θ 旋轉（MG996R 180° 標準伺服）
 *  - 控制夾爪開合（MG996R 180° 標準伺服）
 *  - 限位開關保護 r 齒條兩端行程
 *  - Serial Monitor 手動測試模式
 *
 *  馬達配置：
 *  - MG996R 360°（GPIO 0）：r 齒條伸縮，人眼判斷位置
 *  - MG996R 180°（GPIO 1）：θ 旋轉
 *  - MG996R 180°（GPIO 2）：夾爪開合
 *
 *  指令協議（ESP-NOW 接收 / Serial 手動測試）：
 *    CMD_SERVO_STOP = 0  → 全部停止（360° 停轉，180° 保持）
 *    CMD_R_EXTEND   = 1  → r 齒條伸出（360° 正轉）
 *    CMD_R_RETRACT  = 2  → r 齒條縮回（360° 反轉）
 *    CMD_THETA_POS  = 3  → θ 旋轉（正方向）
 *    CMD_THETA_NEG  = 4  → θ 旋轉（反方向）
 *    CMD_CLAW_OPEN  = 5  → 夾爪張開
 *    CMD_CLAW_CLOSE = 6  → 夾爪閉合
 *    CMD_SERVO_HOME = 7  → 歸位（r 縮回 + 夾爪張開）
 *
 *  Serial 快捷鍵：
 *    'w' = r 伸出       's' = r 縮回
 *    'i' = θ 正轉       'k' = θ 反轉
 *    'o' = 夾爪張開      'p' = 夾爪閉合
 *    'h' = 歸位          '0' = 全部停止
 *    '?' = 顯示狀態
 *
 *  硬體接線：
 *  ESP32-C3 Super Mini
 *  ─────────────────────
 *  GPIO 0 (PWM)  -> MG996R 360°（r 齒條伸縮）── 信號線
 *  GPIO 1 (PWM)  -> MG996R 180°（θ 旋轉）── 信號線
 *  GPIO 2 (PWM)  -> MG996R 180°（夾爪開合）── 信號線
 *
 *  限位開關：
 *  GPIO 4  -> r 齒條伸出極限 (Limit Switch, 常開 NO, 接 GND)
 *  GPIO 5  -> r 齒條縮回極限
 *
 *  注意：
 *  - ESP32-C3 不支援 BluetoothSerial，使用 ESP-NOW
 *  - Servo 供電需 XL4015 5A 降壓模組 5V 輸出（Servo 專用），不可從 C3 取電
 *  - 360° 連續旋轉舵機：write(90) = 停止, write(0) = 全速反轉, write(180) = 全速正轉
 *  - 360° 舵機沒有位置回饋，靠限位開關防止齒條過行程
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// =====================================================
//  ESP-NOW 資料格式（與 protocol.h 一致）
// =====================================================
enum ServoClawCommand {
  CMD_SERVO_STOP = 0,
  CMD_R_EXTEND,        // 1 - r 齒條伸出（360° Servo 正轉）
  CMD_R_RETRACT,       // 2 - r 齒條縮回（360° Servo 反轉）
  CMD_THETA_POS,       // 3 - θ 旋轉（正方向）
  CMD_THETA_NEG,       // 4 - θ 旋轉（反方向）
  CMD_CLAW_OPEN,       // 5 - 夾爪張開
  CMD_CLAW_CLOSE,      // 6 - 夾爪閉合
  CMD_SERVO_HOME,      // 7 - 歸位
};

typedef struct servo_claw_now_message {
  uint8_t command;
  uint8_t speed;
} servo_claw_now_message;

servo_claw_now_message incomingMsg;
bool newDataReceived = false;

// =====================================================
//  Servo 腳位定義
// =====================================================
const int servoR_Pin     = 0;   // MG996R 360° — r 齒條
const int servoTheta_Pin = 1;   // MG996R 180° — θ 旋轉
const int servoClaw_Pin  = 2;   // MG996R 180° — 夾爪開合

Servo servoR;       // 360° 連續旋轉
Servo servoTheta;   // 180° 標準
Servo servoClaw;    // 180° 標準

// =====================================================
//  限位開關腳位
//  接法：GPIO ---- Limit Switch (NO) ---- GND
//  使用 INPUT_PULLUP：未觸發 = HIGH，觸發 = LOW
// =====================================================
const int limitR_Extend  = 4;   // r 齒條伸出極限
const int limitR_Retract = 5;   // r 齒條縮回極限

// =====================================================
//  Servo 角度設定
// =====================================================
// --- 360° 連續旋轉舵機 (r 齒條) ---
// write(90) = 停止, write(0) = 全速反轉, write(180) = 全速正轉
// 實際零點可能有偏移，需要微調
const int R_STOP     = 90;    // 停止
const int R_EXTEND   = 180;   // 正轉（伸出）
const int R_RETRACT  = 0;     // 反轉（縮回）

// --- 180° 標準伺服 (θ 旋轉) ---
const int THETA_MIN     = 0;
const int THETA_MAX     = 180;
const int THETA_CENTER  = 90;
const int THETA_STEP    = 15;   // 每次旋轉步進角度

// --- 180° 標準伺服 (夾爪開合) ---
const int CLAW_OPEN_ANGLE  = 180;  // 夾爪張開角度（需實測調整）
const int CLAW_CLOSE_ANGLE = 0;    // 夾爪閉合角度（需實測調整）

int currentThetaAngle = THETA_CENTER;
int currentClawAngle  = CLAW_OPEN_ANGLE;

// =====================================================
//  r 齒條運行狀態（用於限位開關檢測）
// =====================================================
bool rExtending = false;   // 正在伸出
bool rRetracting = false;  // 正在縮回

// 安全保護：r 齒條最大運行時間
const unsigned long R_MAX_RUN_TIME = 5000;  // 5 秒
unsigned long rStartTime = 0;

// =====================================================
//  ESP-NOW 回調：接收到資料時觸發
// =====================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(servo_claw_now_message)) {
    memcpy(&incomingMsg, data, sizeof(servo_claw_now_message));
    newDataReceived = true;

    Serial.print("ESP-NOW Recv -> CMD: ");
    Serial.print(incomingMsg.command);
    Serial.print(", Speed: ");
    Serial.println(incomingMsg.speed);
  }
}

// =====================================================
//  r 齒條控制（360° 連續旋轉舵機）
// =====================================================

void rExtend() {
  // 檢查限位開關
  if (digitalRead(limitR_Extend) == LOW) {
    Serial.println("!! r Extend Limit Hit -> STOP");
    rStop();
    return;
  }
  servoR.write(R_EXTEND);
  rExtending = true;
  rRetracting = false;
  rStartTime = millis();
  Serial.println("r Extend (rack out)");
}

void rRetract() {
  if (digitalRead(limitR_Retract) == LOW) {
    Serial.println("!! r Retract Limit Hit -> STOP");
    rStop();
    return;
  }
  servoR.write(R_RETRACT);
  rExtending = false;
  rRetracting = true;
  rStartTime = millis();
  Serial.println("r Retract (rack in)");
}

void rStop() {
  servoR.write(R_STOP);
  rExtending = false;
  rRetracting = false;
  Serial.println("r STOP");
}

// =====================================================
//  θ 旋轉控制（180° 標準伺服）
// =====================================================

void thetaPos() {
  currentThetaAngle = constrain(currentThetaAngle + THETA_STEP, THETA_MIN, THETA_MAX);
  servoTheta.write(currentThetaAngle);
  Serial.print("Theta -> ");
  Serial.print(currentThetaAngle);
  Serial.println("°");
}

void thetaNeg() {
  currentThetaAngle = constrain(currentThetaAngle - THETA_STEP, THETA_MIN, THETA_MAX);
  servoTheta.write(currentThetaAngle);
  Serial.print("Theta -> ");
  Serial.print(currentThetaAngle);
  Serial.println("°");
}

// =====================================================
//  夾爪開合控制（180° 標準伺服）
// =====================================================

void clawOpen() {
  currentClawAngle = CLAW_OPEN_ANGLE;
  servoClaw.write(currentClawAngle);
  Serial.print("Claw OPEN -> ");
  Serial.print(currentClawAngle);
  Serial.println("°");
}

void clawClose() {
  currentClawAngle = CLAW_CLOSE_ANGLE;
  servoClaw.write(currentClawAngle);
  Serial.print("Claw CLOSE -> ");
  Serial.print(currentClawAngle);
  Serial.println("°");
}

// =====================================================
//  歸位功能
// =====================================================
void startHome() {
  Serial.println("=== HOME SEQUENCE ===");
  rRetract();     // 齒條縮回
  clawOpen();     // 夾爪張開
  currentThetaAngle = THETA_CENTER;
  servoTheta.write(currentThetaAngle);
  Serial.println("Theta -> CENTER");
  Serial.println("=== HOME DONE ===");
}

// =====================================================
//  全部停止
// =====================================================
void allStop() {
  rStop();
  // 180° 伺服保持目前角度，不回預設
  Serial.println("=== ALL STOP ===");
}

// =====================================================
//  安全保護：r 齒條超時自動停止
// =====================================================
void checkRTimeout() {
  if ((rExtending || rRetracting) && (millis() - rStartTime > R_MAX_RUN_TIME)) {
    Serial.println("!! r TIMEOUT -> Safety STOP");
    rStop();
  }
}

// =====================================================
//  安全保護：即時限位開關檢測
// =====================================================
void checkLimitSwitches() {
  if (rExtending && digitalRead(limitR_Extend) == LOW) {
    Serial.println("!! r Extend Limit -> Emergency STOP");
    rStop();
  }
  if (rRetracting && digitalRead(limitR_Retract) == LOW) {
    Serial.println("!! r Retract Limit -> Emergency STOP");
    rStop();
  }
}

// =====================================================
//  處理指令（ESP-NOW 或 Serial 共用）
// =====================================================
void executeCommand(uint8_t cmd, uint8_t spd) {
  switch (cmd) {
    case CMD_SERVO_STOP:
      allStop();
      break;
    case CMD_R_EXTEND:
      rExtend();
      break;
    case CMD_R_RETRACT:
      rRetract();
      break;
    case CMD_THETA_POS:
      thetaPos();
      break;
    case CMD_THETA_NEG:
      thetaNeg();
      break;
    case CMD_CLAW_OPEN:
      clawOpen();
      break;
    case CMD_CLAW_CLOSE:
      clawClose();
      break;
    case CMD_SERVO_HOME:
      startHome();
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
  delay(500);

  Serial.println("==========================================");
  Serial.println("  ESP32-C3 #3 Servo Claw Controller");
  Serial.println("  MG996R 360°(r) + 180°(θ) + 180°(claw)");
  Serial.println("==========================================");

  // ----- 限位開關設定（內部上拉） -----
  pinMode(limitR_Extend,  INPUT_PULLUP);
  pinMode(limitR_Retract, INPUT_PULLUP);

  // ----- Servo 初始化 -----
  servoR.attach(servoR_Pin);
  servoTheta.attach(servoTheta_Pin);
  servoClaw.attach(servoClaw_Pin);

  // ----- 初始位置 -----
  rStop();                        // 360° 停轉
  servoTheta.write(THETA_CENTER); // θ 居中
  clawOpen();                     // 夾爪張開

  // ----- ESP-NOW 初始化 -----
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("!! ESP-NOW Init FAILED");
  } else {
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW Initialized, waiting for Servo commands...");
  }

  // ----- 印出使用說明 -----
  Serial.println();
  Serial.println("--- Serial Test Commands ---");
  Serial.println("  w = r Extend    s = r Retract");
  Serial.println("  i = Theta (+)   k = Theta (-)");
  Serial.println("  o = Claw Open   p = Claw Close");
  Serial.println("  h = Home        0 = ALL STOP");
  Serial.println("  ? = Show status");
  Serial.println("----------------------------");
  Serial.print("Theta angle: ");
  Serial.println(currentThetaAngle);
  Serial.print("Claw angle: ");
  Serial.println(currentClawAngle);
  Serial.println();
}

// =====================================================
//  Main Loop
// =====================================================
void loop() {
  // ----- 安全保護 -----
  checkRTimeout();
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
        executeCommand(CMD_R_EXTEND, 0);
        break;
      case 's':
        executeCommand(CMD_R_RETRACT, 0);
        break;
      case 'i':
        executeCommand(CMD_THETA_POS, 0);
        break;
      case 'k':
        executeCommand(CMD_THETA_NEG, 0);
        break;
      case 'o':
        executeCommand(CMD_CLAW_OPEN, 0);
        break;
      case 'p':
        executeCommand(CMD_CLAW_CLOSE, 0);
        break;
      case 'h':
        executeCommand(CMD_SERVO_HOME, 0);
        break;
      case '0':
        executeCommand(CMD_SERVO_STOP, 0);
        break;
      case '?':
        Serial.println("--- Status ---");
        Serial.print("  r state: ");
        if (rExtending) Serial.println("EXTENDING");
        else if (rRetracting) Serial.println("RETRACTING");
        else Serial.println("STOPPED");
        Serial.print("  Theta angle: ");
        Serial.print(currentThetaAngle);
        Serial.println("°");
        Serial.print("  Claw angle: ");
        Serial.print(currentClawAngle);
        Serial.println("°");
        Serial.print("  Limit R_Ext: ");
        Serial.println(digitalRead(limitR_Extend) == LOW ? "TRIGGERED" : "OK");
        Serial.print("  Limit R_Ret: ");
        Serial.println(digitalRead(limitR_Retract) == LOW ? "TRIGGERED" : "OK");
        Serial.println("--------------");
        break;
      default:
        if (c != '\n' && c != '\r') {
          Serial.print("Unknown key: ");
          Serial.println(c);
        }
        break;
    }
  }

  delay(10);
}
