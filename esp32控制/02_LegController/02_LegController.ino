/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 腿部子控板
 *  ESP32-C3 #1 LegController
 * =====================================================
 *
 *  功能：
 *  - 透過 ESP-NOW 接收主控板（ESP32）腿部指令
 *  - 控制左右兩顆 JGB37-555 DC 12V 66rpm 直流減速馬達（前後同速、左右反向原地旋轉）
 *  - 使用 BTS7960 (IBT-2) ×2 驅動馬達（MOSFET H-Bridge）
 *  - 僅支援原地旋轉轉向（不做左右輪不同速轉彎，避免重心不穩）
 *  - 供電：腿部獨立 4S 18650 電池組，C3 #1 由同一組電池經 LM2596 降壓供電
 *
 *  指令協議（ESP-NOW 接收 / Serial 手動測試）：
 *    CMD_LEG_STOP    = 0  → 停止
 *    CMD_FORWARD     = 1  → 前進（左右正轉）
 *    CMD_BACKWARD    = 2  → 後退（左右反轉）
 *    CMD_SPIN_LEFT   = 3  → 原地左旋（左反右正）
 *    CMD_SPIN_RIGHT  = 4  → 原地右旋（左正右反）
 *
 *  ⚠ 左右輪不同速轉彎（CMD_LEFT/CMD_RIGHT）已移除，避免重心不穩。
 *
 *  Serial 快捷鍵：
 *    'f'=前進 'b'=後退 'l'=左旋 'r'=右旋
 *    'q'=左旋 'e'=右旋 '0'=停止
 *    '+'=加速 '-'=減速 '?'=狀態
 *
 *  硬體接線（ESP32-C3 Super Mini → BTS7960 ×2）：
 *
 *  【BTS7960 #1 — 左馬達】
 *  GPIO 0  → RPWM (正轉 PWM)
 *  GPIO 1  → LPWM (反轉 PWM)
 *  R_EN / L_EN → 接 3.3V（常開）
 *
 *  【BTS7960 #2 — 右馬達】
 *  GPIO 3  → RPWM (正轉 PWM)
 *  GPIO 4  → LPWM (反轉 PWM)
 *  R_EN / L_EN → 接 3.3V（常開）
 *
 *  GND     → BTS7960 GND（共地）
 *
 *  BTS7960 控制原理：
 *  - 正轉：RPWM = PWM duty, LPWM = 0
 *  - 反轉：RPWM = 0, LPWM = PWM duty
 *  - 停止：RPWM = 0, LPWM = 0（馬達浮接）
 *  - 煞車：RPWM = 255, LPWM = 255（主動煞車）
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "../protocol.h"

const uint8_t ESPNOW_CHANNEL = 1;

leg_now_message incomingMsg;
bool newDataReceived = false;

// =====================================================
//  BTS7960 腳位定義 — ESP32-C3 Super Mini
// =====================================================
// 左馬達 (BTS7960 #1)
const int motorL_RPWM = 0;   // 正轉 PWM
const int motorL_LPWM = 1;   // 反轉 PWM

// 右馬達 (BTS7960 #2)
const int motorR_RPWM = 3;   // 正轉 PWM
const int motorR_LPWM = 4;   // 反轉 PWM

// =====================================================
//  PWM 設定
// =====================================================
const int pwmFreq       = 20000;  // 20kHz (BTS7960 最佳範圍, 超出人耳)
const int pwmResolution = 8;      // 8-bit → 0~255
// LEDC channels: 0=L_RPWM, 1=L_LPWM, 2=R_RPWM, 3=R_LPWM
const int pwmCh_L_RPWM = 0;
const int pwmCh_L_LPWM = 1;
const int pwmCh_R_RPWM = 2;
const int pwmCh_R_LPWM = 3;

// =====================================================
//  速度設定
// =====================================================
int dutyCycle = LEG_DEFAULT_SPEED; // 4S 直供 12V 馬達時先用保守 duty 測試
const int SPEED_STEP = 20;

// 主控板會定期重送持續移動命令；若封包中斷則自動停車。
const unsigned long COMMAND_TIMEOUT_MS = 1200;
unsigned long lastRemoteCommandMs = 0;
bool remoteMotionActive = false;

// =====================================================
//  ESP-NOW 回調：接收到資料時觸發
// =====================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(leg_now_message)) {
    memcpy(&incomingMsg, data, sizeof(leg_now_message));
    newDataReceived = true;
    Serial.print("ESP-NOW Recv -> CMD: ");
    Serial.print(incomingMsg.command);
    Serial.print(", Speed: ");
    Serial.println(incomingMsg.speed);
  }
}

// =====================================================
//  BTS7960 馬達控制函式
// =====================================================

// 設定單顆馬達速度與方向
// speed > 0: 正轉, speed < 0: 反轉, speed = 0: 停止
void setMotor(int rpwmPin, int lpwmPin, int speed) {
  if (speed > 0) {
    ledcWrite(rpwmPin, speed);
    ledcWrite(lpwmPin, 0);
  } else if (speed < 0) {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, -speed);
  } else {
    ledcWrite(rpwmPin, 0);
    ledcWrite(lpwmPin, 0);
  }
}

void stopMotors() {
  setMotor(motorL_RPWM, motorL_LPWM, 0);
  setMotor(motorR_RPWM, motorR_LPWM, 0);
  Serial.println("=== STOP ===");
}

void forward(int spd) {
  setMotor(motorL_RPWM, motorL_LPWM, spd);
  setMotor(motorR_RPWM, motorR_LPWM, spd);
  Serial.print("Forward, speed="); Serial.println(spd);
}

void backward(int spd) {
  setMotor(motorL_RPWM, motorL_LPWM, -spd);
  setMotor(motorR_RPWM, motorR_LPWM, -spd);
  Serial.print("Backward, speed="); Serial.println(spd);
}

void spinLeft(int spd) {
  // 左反轉、右正轉 → 原地左旋
  setMotor(motorL_RPWM, motorL_LPWM, -spd);
  setMotor(motorR_RPWM, motorR_LPWM, spd);
  Serial.print("Spin Left, speed="); Serial.println(spd);
}

void spinRight(int spd) {
  // 左正轉、右反轉 → 原地右旋
  setMotor(motorL_RPWM, motorL_LPWM, spd);
  setMotor(motorR_RPWM, motorR_LPWM, -spd);
  Serial.print("Spin Right, speed="); Serial.println(spd);
}

// =====================================================
//  處理指令
// =====================================================
bool isMotionCommand(uint8_t cmd) {
  return cmd == CMD_FORWARD || cmd == CMD_BACKWARD ||
         cmd == CMD_SPIN_LEFT || cmd == CMD_SPIN_RIGHT;
}

void executeCommand(uint8_t cmd, uint8_t spd, bool fromRemote = false) {
  if (spd > 0) {
    dutyCycle = constrain(spd, SPEED_MIN, SPEED_MAX);
  }

  if (fromRemote) {
    lastRemoteCommandMs = millis();
    remoteMotionActive = isMotionCommand(cmd);
  }

  switch (cmd) {
    case CMD_LEG_STOP:   stopMotors(); break;
    case CMD_FORWARD:    forward(dutyCycle); break;
    case CMD_BACKWARD:   backward(dutyCycle); break;
    case CMD_SPIN_LEFT:  spinLeft(dutyCycle); break;
    case CMD_SPIN_RIGHT: spinRight(dutyCycle); break;
    default:
      Serial.print("Unknown CMD: ");
      Serial.println(cmd);
      break;
  }
}

void checkCommandTimeout() {
  if (!remoteMotionActive) {
    return;
  }

  if (millis() - lastRemoteCommandMs > COMMAND_TIMEOUT_MS) {
    Serial.println("!! Remote command TIMEOUT -> Safety STOP");
    stopMotors();
    remoteMotionActive = false;
  }
}

// =====================================================
//  Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("========================================");
  Serial.println("  ESP32-C3 #1 Leg Controller");
  Serial.println("  Walking Motor Driver (BTS7960 x2)");
  Serial.println("========================================");

  // ----- PWM 設定（4 個 LEDC channel）-----
  ledcAttachChannel(motorL_RPWM, pwmFreq, pwmResolution, pwmCh_L_RPWM);
  ledcAttachChannel(motorL_LPWM, pwmFreq, pwmResolution, pwmCh_L_LPWM);
  ledcAttachChannel(motorR_RPWM, pwmFreq, pwmResolution, pwmCh_R_RPWM);
  ledcAttachChannel(motorR_LPWM, pwmFreq, pwmResolution, pwmCh_R_LPWM);

  // ----- 初始停止 -----
  stopMotors();

  // ----- ESP-NOW 初始化 -----
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t channelResult = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channelResult != ESP_OK) {
    Serial.print("Failed to set WiFi channel, err=");
    Serial.println(channelResult);
  }

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("!! ESP-NOW Init FAILED");
  } else {
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW Initialized, waiting for leg commands...");
  }

  // ----- 使用說明 -----
  Serial.println();
  Serial.println("--- Serial Test Commands ---");
  Serial.println("  f=Forward  b=Backward");
  Serial.println("  l=SpinL    r=SpinR");
  Serial.println("  q=SpinL    e=SpinR");
  Serial.println("  0=STOP     +=SpeedUP  -=SpeedDOWN");
  Serial.println("  ?=Status");
  Serial.println("----------------------------");
  Serial.print("Current speed: ");
  Serial.println(dutyCycle);
  Serial.println();
}

// =====================================================
//  Main Loop
// =====================================================
void loop() {
  // ----- 處理 ESP-NOW 指令 -----
  if (newDataReceived) {
    newDataReceived = false;
    executeCommand(incomingMsg.command, incomingMsg.speed, true);
  }

  checkCommandTimeout();

  // ----- Serial 手動測試 -----
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'f': executeCommand(CMD_FORWARD, 0); break;
      case 'b': executeCommand(CMD_BACKWARD, 0); break;
      case 'l': executeCommand(CMD_SPIN_LEFT, 0); break;
      case 'r': executeCommand(CMD_SPIN_RIGHT, 0); break;
      case 'q': executeCommand(CMD_SPIN_LEFT, 0); break;
      case 'e': executeCommand(CMD_SPIN_RIGHT, 0); break;
      case '0': executeCommand(CMD_LEG_STOP, 0); break;
      case '+':
        dutyCycle = constrain(dutyCycle + SPEED_STEP, SPEED_MIN, SPEED_MAX);
        Serial.print("Speed UP -> "); Serial.println(dutyCycle);
        break;
      case '-':
        dutyCycle = constrain(dutyCycle - SPEED_STEP, SPEED_MIN, SPEED_MAX);
        Serial.print("Speed DOWN -> "); Serial.println(dutyCycle);
        break;
      case '?':
        Serial.println("--- Status ---");
        Serial.print("  Speed: "); Serial.println(dutyCycle);
        Serial.print("  Remote active: "); Serial.println(remoteMotionActive ? "YES" : "NO");
        Serial.print("  Driver: BTS7960 x2");
        Serial.println();
        Serial.println("--------------");
        break;
      default:
        if (c != '\n' && c != '\r') {
          Serial.print("Unknown key: "); Serial.println(c);
        }
        break;
    }
  }

  delay(10);
}
