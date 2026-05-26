/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 大圓盤+z 子控板
 *  子控板：ESP32-C3 #2 Super Mini
 * =====================================================
 *
 *  功能：
 *  - 透過 ESP-NOW 接收主控板（ESP32）指令
 *  - 控制夾爪底部大圓盤旋轉（XD-25GA 370，自鎖）
 *  - 控制 z 方向升降（JGY370 蝸桿馬達，12V，自鎖）
 *  - 使用 BTS7960 (IBT-2) #3 驅動大圓盤馬達
 *  - 使用 BTS7960 (IBT-2) #4 驅動 z 升降馬達
 *  - 超時自動斷電安全保護
 *  - Serial Monitor 手動測試模式
 *
 *  馬達配置：
 *  - 大圓盤 (BTS7960 #3)：XD-25GA 370 — 夾爪底部大圓盤旋轉
 *  - z 升降 (BTS7960 #4)：JGY370 — z 方向升降（蝸桿自鎖）
 *
 *  指令協議（ESP-NOW 接收 / Serial 手動測試）：
 *    'a' = 大圓盤左轉
 *    'd' = 大圓盤右轉
 *    'u' = z 上升
 *    'j' = z 下降
 *    '0' = 全部停止
 *    '+' = 加速
 *    '-' = 減速
 *
 *  硬體接線：
 *  ESP32-C3 Super Mini
 *  ─────────────────────
 *  【BTS7960 #3 — 大圓盤旋轉】
 *  GPIO 0  -> RPWM (正轉 PWM)
 *  GPIO 1  -> LPWM (反轉 PWM)
 *  R_EN / L_EN -> 接 3.3V（常開）
 *
 *  【BTS7960 #4 — z 升降】
 *  GPIO 3  -> RPWM (上升 PWM)
 *  GPIO 4  -> LPWM (下降 PWM)
 *  R_EN / L_EN -> 接 3.3V（常開）
 *
 *  GND     -> GND (共地)
 *
 *  注意：
 *  - BTS7960 #3/#4 R_EN/L_EN 接 3.3V 常開，不佔 GPIO
 *  - XD-25GA 370 減速箱有自鎖效應，斷電後可保持位置
 *  - JGY370 蝸桿自鎖，斷電後不會下滑
 *  - 供電：LM2596 降壓至 5V（MCU 專用），BTS7960 直接接電池 12V
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "../protocol.h"

const uint8_t ESPNOW_CHANNEL = 1;

turntable_z_now_message incomingMsg;
bool newDataReceived = false;
bool espNowReady = false;
uint8_t lastSenderMac[6] = {0};
bool senderKnown = false;

// =====================================================
//  馬達腳位定義 — ESP32-C3 Super Mini
// =====================================================
// 大圓盤旋轉 (BTS7960 #3, XD-25GA 370)
const int motorTurntable_RPWM = 0;   // 正轉 PWM
const int motorTurntable_LPWM = 1;   // 反轉 PWM

// z 升降 (BTS7960 #4, JGY370)
const int motorZ_RPWM = 3;   // 上升 PWM
const int motorZ_LPWM = 4;   // 下降 PWM


// =====================================================
//  PWM 設定
// =====================================================
const int pwmFreq       = 20000;  // 20kHz (BTS7960 最佳範圍)
const int pwmResolution = 8;      // 8-bit -> 0~255
// LEDC channels: 0=T_RPWM, 1=T_LPWM, 2=Z_RPWM, 3=Z_LPWM
const int pwmCh_T_RPWM  = 0;
const int pwmCh_T_LPWM  = 1;
const int pwmCh_Z_RPWM  = 2;
const int pwmCh_Z_LPWM  = 3;

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
unsigned long motorTurntable_startTime = 0;
unsigned long motorZ_startTime = 0;
bool motorTurntable_running = false;
bool motorZ_running = false;
int turntableDirection = 0;  // +1=left, -1=right, 0=stopped (for limit switch check)
int zDirection = 0;          // +1=up, -1=down, 0=stopped

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
    CTRL_TURNTABLE_Z,
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
  if (data == nullptr || len != sizeof(turntable_z_now_message)) {
    Serial.print("Invalid ESP-NOW packet, len=");
    Serial.println(len);
    return;
  }

  memcpy(&incomingMsg, data, sizeof(turntable_z_now_message));

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
//  馬達控制函式
// =====================================================

// --- 大圓盤：左轉 (BTS7960 #3) ---
void turntableLeft() {

  ledcWrite(motorTurntable_RPWM, dutyCycle);
  ledcWrite(motorTurntable_LPWM, 0);

  if (!motorTurntable_running) {
    motorTurntable_running = true;
    motorTurntable_startTime = millis();
  }
  turntableDirection = 1;
  Serial.print("Turntable Left, speed=");
  Serial.println(dutyCycle);
}

// --- 大圓盤：右轉 (BTS7960 #3) ---
void turntableRight() {

  ledcWrite(motorTurntable_RPWM, 0);
  ledcWrite(motorTurntable_LPWM, dutyCycle);

  if (!motorTurntable_running) {
    motorTurntable_running = true;
    motorTurntable_startTime = millis();
  }
  turntableDirection = -1;
  Serial.print("Turntable Right, speed=");
  Serial.println(dutyCycle);
}

// --- 大圓盤停止 ---
void turntableStop() {
  ledcWrite(motorTurntable_RPWM, 0);
  ledcWrite(motorTurntable_LPWM, 0);
  motorTurntable_running = false;
  turntableDirection = 0;
  Serial.println("Turntable STOP");
}

// --- z 方向：上升 (BTS7960 #4) ---
void zUp() {

  ledcWrite(motorZ_RPWM, dutyCycle);
  ledcWrite(motorZ_LPWM, 0);

  if (!motorZ_running) {
    motorZ_running = true;
    motorZ_startTime = millis();
  }
  zDirection = 1;
  Serial.print("Z Up, speed=");
  Serial.println(dutyCycle);
}

// --- z 方向：下降 (BTS7960 #4) ---
void zDown() {

  ledcWrite(motorZ_RPWM, 0);
  ledcWrite(motorZ_LPWM, dutyCycle);

  if (!motorZ_running) {
    motorZ_running = true;
    motorZ_startTime = millis();
  }
  zDirection = -1;
  Serial.print("Z Down, speed=");
  Serial.println(dutyCycle);
}

// --- z 馬達停止 ---
void zStop() {
  ledcWrite(motorZ_RPWM, 0);
  ledcWrite(motorZ_LPWM, 0);
  motorZ_running = false;
  zDirection = 0;
  Serial.println("Motor Z STOP");
}

// --- 全部停止 ---
void allStop() {
  turntableStop();
  zStop();
  Serial.println("=== ALL STOP ===");
}

// =====================================================
//  安全保護：超時自動斷電
// =====================================================
void checkTimeout() {
  if (motorTurntable_running && (millis() - motorTurntable_startTime > MAX_RUN_TIME)) {
    Serial.println("!! Turntable TIMEOUT -> Safety STOP");
    turntableStop();
  }
  if (motorZ_running && (millis() - motorZ_startTime > MAX_RUN_TIME)) {
    Serial.println("!! Motor Z TIMEOUT -> Safety STOP");
    zStop();
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
    case CMD_TZ_STOP:
      allStop();
      break;
    case CMD_TURNTABLE_LEFT:
      turntableLeft();
      break;
    case CMD_TURNTABLE_RIGHT:
      turntableRight();
      break;
    case CMD_Z_UP:
      zUp();
      break;
    case CMD_Z_DOWN:
      zDown();
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

  Serial.println("==========================================");
  Serial.println("  ESP32-C3 #2 Turntable + Z Controller");
  Serial.println("  BTS7960 #3: XD-25GA 370 | BTS7960 #4: JGY370");
  Serial.println("==========================================");

  // ----- 大圓盤 PWM 設定 (BTS7960 #3) -----
  ledcAttachChannel(motorTurntable_RPWM, pwmFreq, pwmResolution, pwmCh_T_RPWM);
  ledcAttachChannel(motorTurntable_LPWM, pwmFreq, pwmResolution, pwmCh_T_LPWM);

  // ----- z 升降 PWM 設定 (BTS7960 #4) -----
  ledcAttachChannel(motorZ_RPWM, pwmFreq, pwmResolution, pwmCh_Z_RPWM);
  ledcAttachChannel(motorZ_LPWM, pwmFreq, pwmResolution, pwmCh_Z_LPWM);



  // ----- 初始停止 -----
  allStop();

  // ----- ESP-NOW 初始化 -----
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
  setupEspNow();

  // ----- 印出使用說明 -----
  Serial.println();
  Serial.println("--- Serial Test Commands ---");
  Serial.println("  a = Turntable Left");
  Serial.println("  d = Turntable Right");
  Serial.println("  u = Z Up");
  Serial.println("  j = Z Down");
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


  // ----- 處理 ESP-NOW 接收的指令 -----
  if (newDataReceived) {
    newDataReceived = false;
    executeCommand(incomingMsg.command, incomingMsg.speed);
  }

  // ----- Serial 手動測試模式 -----
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'a':
        executeCommand(CMD_TURNTABLE_LEFT, 0);
        break;
      case 'd':
        executeCommand(CMD_TURNTABLE_RIGHT, 0);
        break;
      case 'u':
        executeCommand(CMD_Z_UP, 0);
        break;
      case 'j':
        executeCommand(CMD_Z_DOWN, 0);
        break;
      case '0':
        executeCommand(CMD_TZ_STOP, 0);
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
        Serial.print("  Turntable running: ");
        Serial.println(motorTurntable_running ? "YES" : "NO");
        Serial.print("  Motor Z running: ");
        Serial.println(motorZ_running ? "YES" : "NO");

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

  delay(10);  // 降低 CPU 負載
}
