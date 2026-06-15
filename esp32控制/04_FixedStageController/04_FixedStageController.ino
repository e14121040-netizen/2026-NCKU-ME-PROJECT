/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 固定端子控板
 *  子控板：ESP32-C3 #3 Super Mini
 * =====================================================
 *
 *  功能：
 *  - 透過 ESP-NOW 接收主控板（ESP32）指令
 *  - 控制固定端 spin right / left 直流減速馬達
 *  - 控制暫存盒 Gate Open / Close servo
 *  - 固定端 spin 馬達與 Gate 都不跨旋轉關節配線
 *  - spin 馬達與 Gate servo 均有超時自動停止保護
 *
 *  硬體接線：
 *  ESP32-C3 Super Mini
 *  ─────────────────────
 *  GPIO 0 (PWM) -> BTS7960 #3 RPWM（spin left）
 *  GPIO 1 (PWM) -> BTS7960 #3 LPWM（spin right）
 *  BTS7960 VCC / R_EN / L_EN -> 3.3V
 *
 *  GPIO 5 (PWM) -> Gate servo 信號線
 *
 *  GND -> 其他機構電源組 GND（C3、XL4015、LM2596、BTS7960 共地）
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include "../protocol.h"

const uint8_t ESPNOW_CHANNEL = 1;

fixed_stage_now_message incomingMsg;
bool newDataReceived = false;

// =====================================================
//  腳位定義 — ESP32-C3 Super Mini
// =====================================================
const int motorSpin_RPWM = 0;
const int motorSpin_LPWM = 1;
const int servoGate_Pin  = 5;

Servo gateServo;
bool gateServoAttached = false;

// =====================================================
//  Spin PWM 設定
// =====================================================
const int pwmFreq       = 20000;
const int pwmResolution = 8;
const int pwmCh_Spin_RPWM  = 4;
const int pwmCh_Spin_LPWM  = 5;

int dutyCycle = FIXED_SPIN_DEFAULT_SPEED;
const int SPEED_STEP = 20;

// =====================================================
//  Gate servo 設定
// =====================================================
const int GATE_OPEN_ANGLE  = 160;
const int GATE_CLOSE_ANGLE = 20;
const unsigned long GATE_OPEN_HOLD_MS = 700;

int currentGateAngle = GATE_CLOSE_ANGLE;

// =====================================================
//  安全保護
// =====================================================
const unsigned long SPIN_MAX_RUN_TIME = 5000;

unsigned long spinStartTime = 0;
unsigned long gateOpenHoldUntilMs = 0;

bool spinRunning = false;
bool gateOpenHolding = false;
int spinDirection = 0;  // +1=left, -1=right, 0=stopped

// =====================================================
//  ESP-NOW 回調
// =====================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (data == nullptr || len != sizeof(fixed_stage_now_message)) {
    Serial.print("Invalid ESP-NOW packet, len=");
    Serial.println(len);
    return;
  }

  memcpy(&incomingMsg, data, sizeof(fixed_stage_now_message));
  newDataReceived = true;

  Serial.print("ESP-NOW Recv -> CMD: ");
  Serial.print(incomingMsg.command);
  Serial.print(", Speed: ");
  Serial.println(incomingMsg.speed);
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t channelResult = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channelResult != ESP_OK) {
    Serial.print("Failed to set WiFi channel, err=");
    Serial.println(channelResult);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("!! ESP-NOW Init FAILED");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW receiver ready");
}

// =====================================================
//  Spin 馬達控制
// =====================================================
void spinLeft() {
  ledcWrite(motorSpin_RPWM, dutyCycle);
  ledcWrite(motorSpin_LPWM, 0);

  if (!spinRunning) {
    spinRunning = true;
    spinStartTime = millis();
  }
  spinDirection = 1;
  Serial.print("Fixed Spin Left, speed=");
  Serial.println(dutyCycle);
}

void spinRight() {
  ledcWrite(motorSpin_RPWM, 0);
  ledcWrite(motorSpin_LPWM, dutyCycle);

  if (!spinRunning) {
    spinRunning = true;
    spinStartTime = millis();
  }
  spinDirection = -1;
  Serial.print("Fixed Spin Right, speed=");
  Serial.println(dutyCycle);
}

void spinStop() {
  ledcWrite(motorSpin_RPWM, 0);
  ledcWrite(motorSpin_LPWM, 0);
  spinRunning = false;
  spinDirection = 0;
  Serial.println("Fixed Spin STOP");
}

// =====================================================
//  Gate servo 控制
// =====================================================
void attachGateServoIfNeeded() {
  if (!gateServoAttached) {
    gateServo.attach(servoGate_Pin);
    gateServoAttached = true;
    Serial.println("Gate servo PWM enabled");
  }
}

void detachGateServoIfNeeded() {
  if (gateServoAttached) {
    gateServo.detach();
    gateServoAttached = false;
    Serial.println("Gate servo PWM disabled");
  }
}

void gateStop() {
  detachGateServoIfNeeded();
  gateOpenHolding = false;
  Serial.println("Gate STOP");
}

void gateOpen() {
  attachGateServoIfNeeded();
  currentGateAngle = GATE_OPEN_ANGLE;
  gateServo.write(GATE_OPEN_ANGLE);
  gateOpenHolding = true;
  gateOpenHoldUntilMs = millis() + GATE_OPEN_HOLD_MS;
  Serial.print("Gate OPEN -> angle ");
  Serial.println(currentGateAngle);
}

void gateClose() {
  attachGateServoIfNeeded();
  currentGateAngle = GATE_CLOSE_ANGLE;
  gateServo.write(GATE_CLOSE_ANGLE);
  gateOpenHolding = false;
  Serial.print("Gate CLOSE hold -> angle ");
  Serial.println(currentGateAngle);
}

void allStop() {
  spinStop();
  gateStop();
  Serial.println("=== FIXED STAGE ALL STOP ===");
}

// =====================================================
//  安全保護
// =====================================================
void checkSpinTimeout() {
  if (spinRunning && (millis() - spinStartTime > SPIN_MAX_RUN_TIME)) {
    Serial.println("!! Fixed Spin TIMEOUT -> Safety STOP");
    spinStop();
  }
}

void checkGateTimeout() {
  if (gateOpenHolding && millis() >= gateOpenHoldUntilMs) {
    gateOpenHolding = false;
    detachGateServoIfNeeded();
    Serial.println("Gate open hold complete -> PWM disabled");
  }
}

// =====================================================
//  處理指令（ESP-NOW 或 Serial 共用）
// =====================================================
void executeCommand(uint8_t cmd, uint8_t spd) {
  if (spd > 0) {
    dutyCycle = constrain(spd, SPEED_MIN, SPEED_MAX);
  }

  switch (cmd) {
    case CMD_FIXED_STOP:
      allStop();
      break;
    case CMD_FIXED_SPIN_LEFT:
      spinLeft();
      break;
    case CMD_FIXED_SPIN_RIGHT:
      spinRight();
      break;
    case CMD_FIXED_GATE_OPEN:
      gateOpen();
      break;
    case CMD_FIXED_GATE_CLOSE:
      gateClose();
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
  Serial.println("  ESP32-C3 #3 Fixed Stage Controller");
  Serial.println("  Fixed spin motor + gate servo");
  Serial.println("==========================================");

  ledcAttachChannel(motorSpin_RPWM, pwmFreq, pwmResolution, pwmCh_Spin_RPWM);
  ledcAttachChannel(motorSpin_LPWM, pwmFreq, pwmResolution, pwmCh_Spin_LPWM);
  spinStop();

  Serial.println("Gate PWM disabled on boot; waiting for command");

  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
  setupEspNow();

  Serial.println();
  Serial.println("--- Serial Test Commands ---");
  Serial.println("  a = Fixed Spin Left");
  Serial.println("  d = Fixed Spin Right");
  Serial.println("  g = Gate Open");
  Serial.println("  n = Gate Close");
  Serial.println("  0 = ALL STOP");
  Serial.println("  + = Speed UP");
  Serial.println("  - = Speed DOWN");
  Serial.println("  ? = Show status");
  Serial.println("----------------------------");
  Serial.print("Current spin speed: ");
  Serial.println(dutyCycle);
  Serial.println();
}

// =====================================================
//  Main Loop
// =====================================================
void loop() {
  checkSpinTimeout();
  checkGateTimeout();

  if (newDataReceived) {
    newDataReceived = false;
    executeCommand(incomingMsg.command, incomingMsg.speed);
  }

  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'a':
        executeCommand(CMD_FIXED_SPIN_LEFT, 0);
        break;
      case 'd':
        executeCommand(CMD_FIXED_SPIN_RIGHT, 0);
        break;
      case 'g':
        executeCommand(CMD_FIXED_GATE_OPEN, 0);
        break;
      case 'n':
        executeCommand(CMD_FIXED_GATE_CLOSE, 0);
        break;
      case '0':
        executeCommand(CMD_FIXED_STOP, 0);
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
        Serial.println("--- Status ---");
        Serial.print("  Speed: ");
        Serial.println(dutyCycle);
        Serial.print("  Spin running: ");
        Serial.println(spinRunning ? "YES" : "NO");
        Serial.print("  Gate PWM attached: ");
        Serial.println(gateServoAttached ? "YES" : "NO");
        Serial.print("  Gate angle: ");
        Serial.println(currentGateAngle);
        Serial.print("  Gate open hold: ");
        Serial.println(gateOpenHolding ? "YES" : "NO");
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
