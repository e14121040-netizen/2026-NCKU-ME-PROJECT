/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 旋轉端上方機構子控板
 *  子控板：ESP32-C3 #2 Super Mini
 * =====================================================
 *
 *  功能：
 *  - 透過 ESP-NOW 接收主控板（ESP32）指令
 *  - 控制 r 齒條伸縮（MG996R 360° 連續旋轉舵機）
 *  - 控制 θ 旋轉（MG996R 180° 標準伺服）
 *  - 控制夾爪開合（MG996R 180° 標準伺服）
 *  - 控制 z 方向升降（JGY370 蝸桿馬達，12V，自鎖）
 *  - Servo PWM 收到指令後才啟用，STOP 時關閉 PWM
 *  - r 齒條與 z 馬達均有超時自動停止保護
 *
 *  硬體接線：
 *  ESP32-C3 Super Mini
 *  ─────────────────────
 *  GPIO 0 (PWM) -> MG996R 360° r 齒條伸縮信號線
 *  GPIO 1 (PWM) -> MG996R 180° θ 旋轉信號線
 *  GPIO 5 (PWM) -> MG996R 180° 夾爪開合信號線
 *
 *  GPIO 3 (PWM) -> BTS7960 #4 RPWM（z 上升）
 *  GPIO 4 (PWM) -> BTS7960 #4 LPWM（z 下降）
 *  BTS7960 VCC / R_EN / L_EN -> 3.3V
 *
 *  GND -> 其他機構電源組 GND（C3、XL4015、LM2596、BTS7960 共地）
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include "../protocol.h"

const uint8_t ESPNOW_CHANNEL = 1;

rotating_arm_now_message incomingMsg;
bool newDataReceived = false;
bool espNowReady = false;
uint8_t lastSenderMac[6] = {0};
bool senderKnown = false;

// =====================================================
//  腳位定義 — ESP32-C3 Super Mini
// =====================================================
const int servoR_Pin     = 0;  // MG996R 360° — r 齒條
const int servoTheta_Pin = 1;  // MG996R 180° — θ 旋轉
const int servoClaw_Pin  = 5;  // MG996R 180° — 夾爪開合

const int motorZ_RPWM = 3;  // BTS7960 #4 上升 PWM
const int motorZ_LPWM = 4;  // BTS7960 #4 下降 PWM

Servo servoR;
Servo servoTheta;
Servo servoClaw;

bool rServoAttached = false;
bool thetaServoAttached = false;
bool clawServoAttached = false;

// =====================================================
//  Servo 角度與速度設定
// =====================================================
const int R_STOP     = 90;
const int R_EXTEND   = 0;
const int R_RETRACT  = 180;

const int THETA_MIN     = 0;
const int THETA_MAX     = 180;
const int THETA_CENTER  = 90;
const int THETA_STEP    = 15;

const int CLAW_OPEN_ANGLE  = 0;
const int CLAW_CLOSE_ANGLE = 180;
const int CLAW_STEP_DEGREES = 4;
const unsigned long CLAW_STEP_INTERVAL_MS = 20;

int currentThetaAngle = THETA_CENTER;
int currentClawAngle  = CLAW_OPEN_ANGLE;
int targetClawAngle   = CLAW_OPEN_ANGLE;

// =====================================================
//  Z PWM 設定
// =====================================================
const int pwmFreq       = 20000;
const int pwmResolution = 8;
const int pwmCh_Z_RPWM  = 4;
const int pwmCh_Z_LPWM  = 5;

int dutyCycle = Z_DEFAULT_SPEED;
const int SPEED_STEP = 20;

// =====================================================
//  安全保護
// =====================================================
const unsigned long R_MAX_RUN_TIME = 5000;
const unsigned long Z_MAX_RUN_TIME = 5000;
const unsigned long STANDARD_SERVO_HOLD_MS = 700;

unsigned long rStartTime = 0;
unsigned long zStartTime = 0;
unsigned long standardServoDetachAtMs = 0;

bool rExtending = false;
bool rRetracting = false;
bool motorZ_running = false;
bool clawMotionActive = false;
int zDirection = 0;  // +1=up, -1=down, 0=stopped
unsigned long lastClawStepAtMs = 0;

// =====================================================
//  ESP-NOW 回調
// =====================================================
void printMacAddress(const uint8_t *mac) {
  if (mac == nullptr) {
    Serial.print("null");
    return;
  }

  for (int i = 0; i < 6; ++i) {
    if (mac[i] < 16) {
      Serial.print('0');
    }
    Serial.print(mac[i], HEX);
    if (i < 5) {
      Serial.print(':');
    }
  }
}

bool ensurePeerExists(const uint8_t *peerMac) {
  if (peerMac == nullptr) {
    return false;
  }

  if (esp_now_is_peer_exist(peerMac)) {
    return true;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  esp_err_t addResult = esp_now_add_peer(&peerInfo);
  if (addResult != ESP_OK) {
    Serial.print("Failed to add sender peer, err=");
    Serial.println(addResult);
    return false;
  }

  Serial.print("Registered sender peer: ");
  printMacAddress(peerMac);
  Serial.println();
  return true;
}

void sendAck() {
  if (!senderKnown) {
    return;
  }

  if (!ensurePeerExists(lastSenderMac)) {
    return;
  }

  ack_message ack = {
    CTRL_ROTATING_ARM,
    incomingMsg.command,
    incomingMsg.speed,
    ACK_STATUS_OK
  };

  esp_err_t result = esp_now_send(lastSenderMac, (uint8_t *)&ack, sizeof(ack));
  if (result != ESP_OK) {
    Serial.print("ACK send failed, err=");
    Serial.println(result);
  }
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (data == nullptr || len != sizeof(rotating_arm_now_message)) {
    Serial.print("Invalid ESP-NOW packet, len=");
    Serial.println(len);
    return;
  }

  memcpy(&incomingMsg, data, sizeof(rotating_arm_now_message));

  if (info != nullptr && info->src_addr != nullptr) {
    memcpy(lastSenderMac, info->src_addr, 6);
    senderKnown = true;
  }

  newDataReceived = true;

  Serial.print("Received command=");
  Serial.print(incomingMsg.command);
  Serial.print(", speed=");
  Serial.print(incomingMsg.speed);
  Serial.print(", from=");
  printMacAddress(lastSenderMac);
  Serial.println();

  sendAck();
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
  espNowReady = true;
  Serial.println("ESP-NOW receiver ready");
}

// =====================================================
//  Servo attach / detach
// =====================================================
void attachRServoIfNeeded() {
  if (!rServoAttached) {
    servoR.attach(servoR_Pin);
    rServoAttached = true;
    Serial.println("r servo PWM enabled");
  }
}

void detachRServoIfNeeded() {
  if (rServoAttached) {
    servoR.detach();
    rServoAttached = false;
    Serial.println("r servo PWM disabled");
  }
}

void attachThetaServoIfNeeded() {
  if (!thetaServoAttached) {
    servoTheta.attach(servoTheta_Pin);
    thetaServoAttached = true;
    Serial.println("Theta servo PWM enabled");
  }
}

void detachThetaServoIfNeeded() {
  if (thetaServoAttached) {
    servoTheta.detach();
    thetaServoAttached = false;
    Serial.println("Theta servo PWM disabled");
  }
}

void attachClawServoIfNeeded() {
  if (!clawServoAttached) {
    servoClaw.attach(servoClaw_Pin);
    clawServoAttached = true;
    Serial.println("Claw servo PWM enabled");
  }
}

void detachClawServoIfNeeded() {
  if (clawServoAttached) {
    servoClaw.detach();
    clawServoAttached = false;
    Serial.println("Claw servo PWM disabled");
  }
}

void scheduleStandardServoDetach() {
  standardServoDetachAtMs = millis() + STANDARD_SERVO_HOLD_MS;
}

void detachAllServoOutputs() {
  detachRServoIfNeeded();
  detachThetaServoIfNeeded();
  detachClawServoIfNeeded();
  rExtending = false;
  rRetracting = false;
  clawMotionActive = false;
  Serial.println("Rotating arm servo PWM disabled");
}

// =====================================================
//  r 齒條控制
// =====================================================
void rExtend() {
  attachRServoIfNeeded();
  servoR.write(R_EXTEND);
  rExtending = true;
  rRetracting = false;
  rStartTime = millis();
  Serial.println("r Extend (rack out)");
}

void rRetract() {
  attachRServoIfNeeded();
  servoR.write(R_RETRACT);
  rExtending = false;
  rRetracting = true;
  rStartTime = millis();
  Serial.println("r Retract (rack in)");
}

void rStop() {
  detachRServoIfNeeded();
  rExtending = false;
  rRetracting = false;
  Serial.println("r STOP");
}

// =====================================================
//  θ 旋轉控制
// =====================================================
void thetaPos() {
  attachThetaServoIfNeeded();
  currentThetaAngle = constrain(currentThetaAngle + THETA_STEP, THETA_MIN, THETA_MAX);
  servoTheta.write(currentThetaAngle);
  scheduleStandardServoDetach();
  Serial.print("Theta -> ");
  Serial.println(currentThetaAngle);
}

void thetaNeg() {
  attachThetaServoIfNeeded();
  currentThetaAngle = constrain(currentThetaAngle - THETA_STEP, THETA_MIN, THETA_MAX);
  servoTheta.write(currentThetaAngle);
  scheduleStandardServoDetach();
  Serial.print("Theta -> ");
  Serial.println(currentThetaAngle);
}

// =====================================================
//  夾爪控制
// =====================================================
void startClawMotion(int targetAngle, const char *label) {
  attachClawServoIfNeeded();
  targetClawAngle = constrain(targetAngle, 0, 180);
  standardServoDetachAtMs = 0;

  servoClaw.write(currentClawAngle);
  clawMotionActive = (currentClawAngle != targetClawAngle);
  lastClawStepAtMs = millis();

  Serial.print("Claw ");
  Serial.print(label);
  Serial.print(" target -> ");
  Serial.println(targetClawAngle);

  if (!clawMotionActive) {
    scheduleStandardServoDetach();
  }
}

void checkClawMotion() {
  if (!clawMotionActive) {
    return;
  }

  unsigned long now = millis();
  if (now - lastClawStepAtMs < CLAW_STEP_INTERVAL_MS) {
    return;
  }

  int delta = targetClawAngle - currentClawAngle;
  if (abs(delta) <= CLAW_STEP_DEGREES) {
    currentClawAngle = targetClawAngle;
  } else {
    currentClawAngle += (delta > 0) ? CLAW_STEP_DEGREES : -CLAW_STEP_DEGREES;
  }

  servoClaw.write(currentClawAngle);
  lastClawStepAtMs = now;

  if (currentClawAngle == targetClawAngle) {
    clawMotionActive = false;
    scheduleStandardServoDetach();
    Serial.print("Claw reached -> ");
    Serial.println(currentClawAngle);
  }
}

void clawOpen() {
  startClawMotion(CLAW_OPEN_ANGLE, "OPEN");
}

void clawClose() {
  startClawMotion(CLAW_CLOSE_ANGLE, "CLOSE");
}

// =====================================================
//  Z 馬達控制
// =====================================================
void zUp() {
  ledcWrite(motorZ_RPWM, dutyCycle);
  ledcWrite(motorZ_LPWM, 0);

  if (!motorZ_running) {
    motorZ_running = true;
    zStartTime = millis();
  }
  zDirection = 1;
  Serial.print("Z Up, speed=");
  Serial.println(dutyCycle);
}

void zDown() {
  ledcWrite(motorZ_RPWM, 0);
  ledcWrite(motorZ_LPWM, dutyCycle);

  if (!motorZ_running) {
    motorZ_running = true;
    zStartTime = millis();
  }
  zDirection = -1;
  Serial.print("Z Down, speed=");
  Serial.println(dutyCycle);
}

void zStop() {
  ledcWrite(motorZ_RPWM, 0);
  ledcWrite(motorZ_LPWM, 0);
  motorZ_running = false;
  zDirection = 0;
  Serial.println("Motor Z STOP");
}

void startHome() {
  Serial.println("=== ROTATING ARM HOME ===");
  rRetract();
  clawOpen();
  attachThetaServoIfNeeded();
  currentThetaAngle = THETA_CENTER;
  servoTheta.write(currentThetaAngle);
  scheduleStandardServoDetach();
  Serial.println("Theta -> CENTER");
  Serial.println("=== ROTATING ARM HOME QUEUED ===");
}

void allStop() {
  zStop();
  rStop();
  Serial.println("=== ROTATING ARM R/Z STOP ===");
}

// =====================================================
//  安全保護
// =====================================================
void checkRTimeout() {
  if ((rExtending || rRetracting) && (millis() - rStartTime > R_MAX_RUN_TIME)) {
    Serial.println("!! r TIMEOUT -> Safety STOP");
    rStop();
  }
}

void checkZTimeout() {
  if (motorZ_running && (millis() - zStartTime > Z_MAX_RUN_TIME)) {
    Serial.println("!! Motor Z TIMEOUT -> Safety STOP");
    zStop();
  }
}

void checkStandardServoDetachTimeout() {
  if (standardServoDetachAtMs == 0) {
    return;
  }

  if ((long)(millis() - standardServoDetachAtMs) >= 0) {
    detachThetaServoIfNeeded();
    if (!clawMotionActive) {
      detachClawServoIfNeeded();
    }
    standardServoDetachAtMs = 0;
    if (clawMotionActive) {
      Serial.println("Theta servo PWM hold elapsed; claw ramp still active");
    } else {
      Serial.println("Theta/Claw servo PWM hold elapsed");
    }
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
    case CMD_ARM_STOP:
      allStop();
      break;
    case CMD_ARM_R_EXTEND:
      rExtend();
      break;
    case CMD_ARM_R_RETRACT:
      rRetract();
      break;
    case CMD_ARM_R_STOP:
      rStop();
      break;
    case CMD_ARM_THETA_POS:
      thetaPos();
      break;
    case CMD_ARM_THETA_NEG:
      thetaNeg();
      break;
    case CMD_ARM_CLAW_OPEN:
      clawOpen();
      break;
    case CMD_ARM_CLAW_CLOSE:
      clawClose();
      break;
    case CMD_ARM_Z_UP:
      zUp();
      break;
    case CMD_ARM_Z_DOWN:
      zDown();
      break;
    case CMD_ARM_HOME:
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
  Serial.println("  ESP32-C3 #2 Rotating Arm Controller");
  Serial.println("  Upper servos + JGY370 Z motor");
  Serial.println("==========================================");

  ledcAttachChannel(motorZ_RPWM, pwmFreq, pwmResolution, pwmCh_Z_RPWM);
  ledcAttachChannel(motorZ_LPWM, pwmFreq, pwmResolution, pwmCh_Z_LPWM);
  zStop();

  Serial.println("r/Theta/Claw PWM disabled on boot; waiting for command");

  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
  setupEspNow();

  Serial.println();
  Serial.println("--- Serial Test Commands ---");
  Serial.println("  w = r Extend    s = r Retract   x = r STOP");
  Serial.println("  i = Theta (+)   k = Theta (-)");
  Serial.println("  o = Claw Open   p = Claw Close");
  Serial.println("  u = Z Up        j = Z Down");
  Serial.println("  h = Home        0 = ALL STOP");
  Serial.println("  + = Speed UP    - = Speed DOWN");
  Serial.println("  ? = Show status");
  Serial.println("----------------------------");
  Serial.print("Current Z speed: ");
  Serial.println(dutyCycle);
  Serial.println();
}

// =====================================================
//  Main Loop
// =====================================================
void loop() {
  checkRTimeout();
  checkZTimeout();
  checkClawMotion();
  checkStandardServoDetachTimeout();

  if (newDataReceived) {
    newDataReceived = false;
    executeCommand(incomingMsg.command, incomingMsg.speed);
  }

  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'w':
        executeCommand(CMD_ARM_R_EXTEND, 0);
        break;
      case 's':
        executeCommand(CMD_ARM_R_RETRACT, 0);
        break;
      case 'x':
        executeCommand(CMD_ARM_R_STOP, 0);
        break;
      case 'i':
        executeCommand(CMD_ARM_THETA_POS, 0);
        break;
      case 'k':
        executeCommand(CMD_ARM_THETA_NEG, 0);
        break;
      case 'o':
        executeCommand(CMD_ARM_CLAW_OPEN, 0);
        break;
      case 'p':
        executeCommand(CMD_ARM_CLAW_CLOSE, 0);
        break;
      case 'u':
        executeCommand(CMD_ARM_Z_UP, 0);
        break;
      case 'j':
        executeCommand(CMD_ARM_Z_DOWN, 0);
        break;
      case 'h':
        executeCommand(CMD_ARM_HOME, 0);
        break;
      case '0':
        executeCommand(CMD_ARM_STOP, 0);
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
        Serial.print("  r state: ");
        if (rExtending) Serial.println("EXTENDING");
        else if (rRetracting) Serial.println("RETRACTING");
        else Serial.println("STOPPED");
        Serial.print("  Theta angle: ");
        Serial.println(currentThetaAngle);
        Serial.print("  Claw angle: ");
        Serial.println(currentClawAngle);
        Serial.print("  Z running: ");
        Serial.println(motorZ_running ? "YES" : "NO");
        Serial.print("  Z speed: ");
        Serial.println(dutyCycle);
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
