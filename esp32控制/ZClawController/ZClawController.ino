/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 z/夾爪子控板
 *  ESP32-C3 #3 ZClawController
 * =====================================================
 *
 *  功能：
 *  - 透過 ESP-NOW 接收主控板（ESP32）z/夾爪指令
 *  - 控制 z 方向升降 DC 馬達（經 L298N 驅動）
 *  - 控制夾爪伺服馬達（GPIO PWM 直接驅動）
 *  - 控制蓋板伺服馬達（GPIO PWM 直接驅動）
 *  - 限位開關 (Limit Switch) 安全保護
 *  - 超時自動斷電（防止馬達卡死燒毀）
 *  - Serial Monitor 手動測試模式
 *
 *  座標系統：
 *  - z   ：垂直升降（DC 馬達經 L298N 驅動）
 *  - 夾爪 ：伺服馬達控制開合（0°~180°）
 *  - 蓋板 ：伺服馬達控制承物台蓋板開合（0°~90°）
 *
 *  指令協議（ESP-NOW 接收 / Serial 手動測試）：
 *    CMD_ZCLAW_STOP    = 0  → 全部停止（DC 馬達停止，伺服保持）
 *    CMD_Z_UP          = 1  → z 上升
 *    CMD_Z_DOWN        = 2  → z 下降
 *    CMD_CLAW_OPEN     = 3  → 夾爪張開（伺服 → 180°）
 *    CMD_CLAW_CLOSE    = 4  → 夾爪閉合（伺服 → 0°）
 *    CMD_LID_OPEN      = 5  → 蓋板開（伺服 → 90°）
 *    CMD_LID_CLOSE     = 6  → 蓋板關（伺服 → 0°）
 *    CMD_HOME          = 7  → 歸位（z 下降到底 + 夾爪張開 + 蓋板關）
 *
 *  Serial 快捷鍵：
 *    'u' = z 上升      'j' = z 下降
 *    'o' = 夾爪張開    'p' = 夾爪閉合
 *    't' = 蓋板開      'y' = 蓋板關
 *    'h' = 歸位        '0' = 全部停止
 *    '+' = 加速        '-' = 減速
 *    '1'~'9' = 夾爪角度 (20°~180°，步進 20°)
 *    '?' = 顯示狀態
 *
 *  硬體接線：
 *  ESP32-C3 Super Mini   ->   L298N #3
 *  ─────────────────────────────────
 *  GPIO 0  -> IN1 (z 升降馬達)
 *  GPIO 1  -> IN2
 *  GPIO 3  -> ENA (PWM)
 *  GND     -> GND (共地)
 *
 *  伺服馬達：
 *  GPIO 4 (PWM)  -> 夾爪伺服馬達（信號線），V+ 接 5V 電源
 *  GPIO 5 (PWM)  -> 蓋板伺服馬達（信號線），V+ 接 5V 電源
 *
 *  限位開關：
 *  GPIO 7  -> z 上升極限 (Limit Switch, 常開 NO, 接 GND)
 *  GPIO 8  -> z 下降極限
 *
 *  注意：
 *  - ESP32-C3 不支援 BluetoothSerial，使用 ESP-NOW
 *  - L298N ENA 跳線帽要拔掉才能用 PWM 調速
 *  - 伺服馬達供電需 5V 穩壓電源，不可直接從 C3 取電
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// =====================================================
//  ESP-NOW 資料格式（與主控板 MainController 一致）
// =====================================================
enum ZClawCommand {
  CMD_ZCLAW_STOP = 0,
  CMD_Z_UP,           // z 上升
  CMD_Z_DOWN,         // z 下降
  CMD_CLAW_OPEN,      // 夾爪張開（伺服）
  CMD_CLAW_CLOSE,     // 夾爪閉合（伺服）
  CMD_LID_OPEN,       // 蓋板開（伺服）
  CMD_LID_CLOSE,      // 蓋板關（伺服）
  CMD_HOME,           // 歸位
};

typedef struct zclaw_now_message {
  uint8_t command;
  uint8_t speed;      // DC 馬達速度 or 伺服角度
} zclaw_now_message;

zclaw_now_message incomingMsg;
bool newDataReceived = false;

// =====================================================
//  L298N 腳位定義 — ESP32-C3 Super Mini
// =====================================================
// z 方向升降馬達
const int motorZ_Pin1 = 0;
const int motorZ_Pin2 = 1;
const int motorZ_EN   = 3;   // PWM 調速

// =====================================================
//  伺服馬達腳位
// =====================================================
const int servoClawPin = 4;  // 夾爪伺服
const int servoLidPin  = 5;  // 蓋板伺服

Servo servoClaw;
Servo servoLid;

// =====================================================
//  限位開關腳位
//  接法：GPIO ---- Limit Switch (NO) ---- GND
//  使用 INPUT_PULLUP：未觸發 = HIGH，觸發 = LOW
// =====================================================
const int limitZ_Up   = 7;   // z 上升極限
const int limitZ_Down = 8;   // z 下降極限

// =====================================================
//  PWM 設定（DC 馬達）
// =====================================================
const int pwmFreq       = 30000;
const int pwmResolution = 8;      // 8-bit -> 0~255
const int pwmChannel_Z  = 0;      // z 馬達 PWM 通道

// =====================================================
//  速度設定
// =====================================================
int dutyCycle = 180;              // 預設速度 (0~255)
const int SPEED_MIN  = 80;       // 最低可動速度
const int SPEED_MAX  = 255;      // 最高速度
const int SPEED_STEP = 20;       // 加減速步進

// =====================================================
//  伺服馬達角度預設值
// =====================================================
const int CLAW_OPEN_ANGLE  = 180;  // 夾爪張開角度
const int CLAW_CLOSE_ANGLE = 0;    // 夾爪閉合角度
const int LID_OPEN_ANGLE   = 90;   // 蓋板開角度
const int LID_CLOSE_ANGLE  = 0;    // 蓋板關角度

int currentClawAngle = CLAW_OPEN_ANGLE;  // 目前夾爪角度
int currentLidAngle  = LID_CLOSE_ANGLE;  // 目前蓋板角度

// =====================================================
//  安全保護：最大通電時間 (ms)
//  超過此時間自動斷電，防止馬達卡死燒毀
// =====================================================
const unsigned long MAX_RUN_TIME = 5000;  // 5 秒
unsigned long motorZ_startTime = 0;
bool motorZ_running = false;

// =====================================================
//  歸位狀態機
// =====================================================
enum HomeState {
  HOME_IDLE = 0,
  HOME_LOWERING,      // 正在下降到底
  HOME_OPEN_CLAW,     // 張開夾爪
  HOME_CLOSE_LID,     // 關閉蓋板
  HOME_DONE,          // 歸位完成
};

HomeState homeState = HOME_IDLE;

// =====================================================
//  ESP-NOW 回調：接收到資料時觸發
// =====================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(zclaw_now_message)) {
    memcpy(&incomingMsg, data, sizeof(zclaw_now_message));
    newDataReceived = true;

    // Debug 輸出
    Serial.print("ESP-NOW Recv -> CMD: ");
    Serial.print(incomingMsg.command);
    Serial.print(", Speed: ");
    Serial.println(incomingMsg.speed);
  }
}

// =====================================================
//  z 馬達控制函式
// =====================================================

// --- z 方向：上升 ---
void zUp() {
  // 檢查限位開關
  if (digitalRead(limitZ_Up) == LOW) {
    Serial.println("!! Z Up Limit Hit -> STOP");
    zStop();
    return;
  }
  digitalWrite(motorZ_Pin1, HIGH);
  digitalWrite(motorZ_Pin2, LOW);
  ledcWrite(motorZ_EN, dutyCycle);

  if (!motorZ_running) {
    motorZ_running = true;
    motorZ_startTime = millis();
  }
  Serial.print("Z Up, speed=");
  Serial.println(dutyCycle);
}

// --- z 方向：下降 ---
void zDown() {
  // 檢查限位開關
  if (digitalRead(limitZ_Down) == LOW) {
    Serial.println("!! Z Down Limit Hit -> STOP");
    zStop();
    return;
  }
  digitalWrite(motorZ_Pin1, LOW);
  digitalWrite(motorZ_Pin2, HIGH);
  ledcWrite(motorZ_EN, dutyCycle);

  if (!motorZ_running) {
    motorZ_running = true;
    motorZ_startTime = millis();
  }
  Serial.print("Z Down, speed=");
  Serial.println(dutyCycle);
}

// --- z 馬達停止 ---
void zStop() {
  digitalWrite(motorZ_Pin1, LOW);
  digitalWrite(motorZ_Pin2, LOW);
  ledcWrite(motorZ_EN, 0);
  motorZ_running = false;
  Serial.println("Motor Z STOP");
}

// =====================================================
//  伺服馬達控制函式
// =====================================================

// --- 夾爪控制 ---
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

void clawSetAngle(int angle) {
  currentClawAngle = constrain(angle, 0, 180);
  servoClaw.write(currentClawAngle);
  Serial.print("Claw Angle -> ");
  Serial.print(currentClawAngle);
  Serial.println("°");
}

// --- 蓋板控制 ---
void lidOpen() {
  currentLidAngle = LID_OPEN_ANGLE;
  servoLid.write(currentLidAngle);
  Serial.print("Lid OPEN -> ");
  Serial.print(currentLidAngle);
  Serial.println("°");
}

void lidClose() {
  currentLidAngle = LID_CLOSE_ANGLE;
  servoLid.write(currentLidAngle);
  Serial.print("Lid CLOSE -> ");
  Serial.print(currentLidAngle);
  Serial.println("°");
}

// =====================================================
//  歸位功能
// =====================================================
void startHome() {
  Serial.println("=== HOME SEQUENCE START ===");
  homeState = HOME_LOWERING;
  zDown();  // 開始下降
}

void updateHome() {
  switch (homeState) {
    case HOME_LOWERING:
      // 等待觸及下極限或超時
      if (digitalRead(limitZ_Down) == LOW || !motorZ_running) {
        zStop();
        homeState = HOME_OPEN_CLAW;
        Serial.println("Home: Z at bottom, opening claw...");
        clawOpen();
        // 直接進入下一步
        homeState = HOME_CLOSE_LID;
        Serial.println("Home: Closing lid...");
        lidClose();
        homeState = HOME_DONE;
        Serial.println("=== HOME SEQUENCE COMPLETE ===");
        homeState = HOME_IDLE;
      }
      break;

    default:
      break;
  }
}

// =====================================================
//  全部停止
// =====================================================
void allStop() {
  zStop();
  homeState = HOME_IDLE;  // 取消歸位
  Serial.println("=== ALL STOP ===");
  // 注意：伺服馬達保持目前角度，不回到預設位置
}

// =====================================================
//  安全保護：超時自動斷電
// =====================================================
void checkTimeout() {
  if (motorZ_running && (millis() - motorZ_startTime > MAX_RUN_TIME)) {
    Serial.println("!! Motor Z TIMEOUT -> Safety STOP");
    zStop();
    if (homeState == HOME_LOWERING) {
      homeState = HOME_IDLE;
      Serial.println("!! Home aborted due to timeout");
    }
  }
}

// =====================================================
//  安全保護：即時限位開關檢測
// =====================================================
void checkLimitSwitches() {
  if (motorZ_running) {
    // 正在上升時，檢查上升極限
    if (digitalRead(motorZ_Pin1) == HIGH && digitalRead(limitZ_Up) == LOW) {
      Serial.println("!! Z Up Limit -> Emergency STOP");
      zStop();
    }
    // 正在下降時，檢查下降極限
    if (digitalRead(motorZ_Pin2) == HIGH && digitalRead(limitZ_Down) == LOW) {
      Serial.println("!! Z Down Limit -> Emergency STOP");
      zStop();
      if (homeState == HOME_LOWERING) {
        // 歸位過程中觸及下極限，繼續歸位流程
        homeState = HOME_OPEN_CLAW;
      }
    }
  }
}

// =====================================================
//  處理指令（ESP-NOW 或 Serial 共用）
// =====================================================
void executeCommand(uint8_t cmd, uint8_t spd) {
  // 如果 ESP-NOW 帶了速度值且為 DC 馬達指令，使用它
  if (spd > 0 && (cmd == CMD_Z_UP || cmd == CMD_Z_DOWN)) {
    dutyCycle = constrain(spd, SPEED_MIN, SPEED_MAX);
  }

  switch (cmd) {
    case CMD_ZCLAW_STOP:
      allStop();
      break;
    case CMD_Z_UP:
      homeState = HOME_IDLE;  // 取消正在進行的歸位
      zUp();
      break;
    case CMD_Z_DOWN:
      homeState = HOME_IDLE;
      zDown();
      break;
    case CMD_CLAW_OPEN:
      clawOpen();
      break;
    case CMD_CLAW_CLOSE:
      clawClose();
      break;
    case CMD_LID_OPEN:
      lidOpen();
      break;
    case CMD_LID_CLOSE:
      lidClose();
      break;
    case CMD_HOME:
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
  delay(500);  // 等 Serial 穩定

  Serial.println("========================================");
  Serial.println("  ESP32-C3 #3 ZClaw Controller");
  Serial.println("  Z Lift + Claw + Lid (L298N + Servo)");
  Serial.println("========================================");

  // ----- DC 馬達腳位設定 -----
  pinMode(motorZ_Pin1, OUTPUT);
  pinMode(motorZ_Pin2, OUTPUT);
  pinMode(motorZ_EN,   OUTPUT);

  // ----- 限位開關設定（內部上拉） -----
  pinMode(limitZ_Up,   INPUT_PULLUP);
  pinMode(limitZ_Down, INPUT_PULLUP);

  // ----- ESP32 PWM (LEDC) 設定 — DC 馬達 -----
  ledcAttachChannel(motorZ_EN, pwmFreq, pwmResolution, pwmChannel_Z);

  // ----- 伺服馬達初始化 -----
  servoClaw.attach(servoClawPin);
  servoLid.attach(servoLidPin);

  // ----- 初始位置：夾爪張開、蓋板關 -----
  clawOpen();
  lidClose();
  zStop();

  // ----- ESP-NOW 初始化 -----
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("!! ESP-NOW Init FAILED");
    // 繼續執行，仍可透過 Serial 測試
  } else {
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW Initialized, waiting for ZClaw commands...");
  }

  // ----- 印出使用說明 -----
  Serial.println();
  Serial.println("--- Serial Test Commands ---");
  Serial.println("  u = Z Up        j = Z Down");
  Serial.println("  o = Claw Open   p = Claw Close");
  Serial.println("  t = Lid Open    y = Lid Close");
  Serial.println("  h = Home        0 = ALL STOP");
  Serial.println("  + = Speed UP    - = Speed DOWN");
  Serial.println("  1~9 = Claw angle (20~180 deg)");
  Serial.println("  ? = Show status");
  Serial.println("----------------------------");
  Serial.print("Current speed: ");
  Serial.println(dutyCycle);
  Serial.print("Claw angle: ");
  Serial.println(currentClawAngle);
  Serial.print("Lid angle: ");
  Serial.println(currentLidAngle);
  Serial.println();
}

// =====================================================
//  Main Loop
// =====================================================
void loop() {
  // ----- 安全保護 -----
  checkTimeout();
  checkLimitSwitches();

  // ----- 歸位狀態機更新 -----
  if (homeState != HOME_IDLE) {
    updateHome();
  }

  // ----- 處理 ESP-NOW 接收的指令 -----
  if (newDataReceived) {
    newDataReceived = false;
    executeCommand(incomingMsg.command, incomingMsg.speed);
  }

  // ----- Serial 手動測試模式 -----
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'u':
        executeCommand(CMD_Z_UP, 0);
        break;
      case 'j':
        executeCommand(CMD_Z_DOWN, 0);
        break;
      case 'o':
        executeCommand(CMD_CLAW_OPEN, 0);
        break;
      case 'p':
        executeCommand(CMD_CLAW_CLOSE, 0);
        break;
      case 't':
        executeCommand(CMD_LID_OPEN, 0);
        break;
      case 'y':
        executeCommand(CMD_LID_CLOSE, 0);
        break;
      case 'h':
        executeCommand(CMD_HOME, 0);
        break;
      case '0':
        executeCommand(CMD_ZCLAW_STOP, 0);
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
      // 數字鍵快速設定夾爪角度：1=20°, 2=40°, ..., 9=180°
      case '1': clawSetAngle(20);  break;
      case '2': clawSetAngle(40);  break;
      case '3': clawSetAngle(60);  break;
      case '4': clawSetAngle(80);  break;
      case '5': clawSetAngle(100); break;
      case '6': clawSetAngle(120); break;
      case '7': clawSetAngle(140); break;
      case '8': clawSetAngle(160); break;
      case '9': clawSetAngle(180); break;
      case '?':
        // 印出目前狀態
        Serial.println("--- Status ---");
        Serial.print("  Z Speed: ");
        Serial.println(dutyCycle);
        Serial.print("  Motor Z running: ");
        Serial.println(motorZ_running ? "YES" : "NO");
        Serial.print("  Claw angle: ");
        Serial.print(currentClawAngle);
        Serial.println("°");
        Serial.print("  Lid angle: ");
        Serial.print(currentLidAngle);
        Serial.println("°");
        Serial.print("  Home state: ");
        Serial.println(homeState);
        Serial.print("  Limit Z_Up: ");
        Serial.println(digitalRead(limitZ_Up) == LOW ? "TRIGGERED" : "OK");
        Serial.print("  Limit Z_Down: ");
        Serial.println(digitalRead(limitZ_Down) == LOW ? "TRIGGERED" : "OK");
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
