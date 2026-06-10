/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 主控板
 *  ESP32 MainController (BLE 版)
 * =====================================================
 *
 *  功能：
 *  - 透過 BLE UART 接收手機端指令
 *  - 透過 ESP-NOW 分發到 3 塊 ESP32-C3 子控板
 *  - 支援全域急停與分路停止
 *  - 解析共用 ACK 格式（目前 C3 #2 有回 ACK）
 *
 *  正式手機控制端：
 *    BLE Controller – Arduino ESP32
 *    Device name: ESP32_MainController
 */

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

#include "../protocol.h"

const uint8_t ESPNOW_CHANNEL = 1;
const unsigned long COMMAND_KEEPALIVE_INTERVAL_MS = 250;

// =====================================================
//  子控板 MAC 地址
//  將 0xFF 佔位符替換為實際 MAC 地址
// =====================================================
uint8_t Leg_Address[] = {0x58, 0x8C, 0x81, 0x9D, 0xF6, 0x90};          // C3 #1 腿部
uint8_t rotatingArm_Address[] = {0x10, 0xB4, 0x1D, 0x1C, 0xD1, 0x28};  // C3 #2 旋轉端上方機構 + Z
uint8_t fixedStage_Address[] = {0x58, 0x8C, 0x81, 0xA1, 0x30, 0xD0};   // C3 #3 固定端暫存盒 + Spin

static const char *BLE_DEVICE_NAME = "ESP32_MainController";
static const char *SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *CHARACTERISTIC_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *CHARACTERISTIC_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

BLECharacteristic *txCharacteristic = nullptr;
bool bleClientConnected = false;
bool lastBleClientConnected = false;

uint8_t currentSpeed = LEG_DEFAULT_SPEED;
unsigned long lastLegSendMs = 0;

uint8_t legCommand = CMD_LEG_STOP;
uint8_t legSpeed = 0;
bool legDirty = false;

uint8_t rotatingArmCommand = CMD_ARM_STOP;
uint8_t rotatingArmSpeed = 0;
bool rotatingArmDirty = false;

uint8_t fixedStageCommand = CMD_FIXED_STOP;
uint8_t fixedStageSpeed = 0;
bool fixedStageDirty = false;

// =====================================================
//  工具函式
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

bool isPlaceholderMac(const uint8_t *mac) {
  if (mac == nullptr) {
    return true;
  }

  for (int i = 0; i < 6; ++i) {
    if (mac[i] != 0xFF) {
      return false;
    }
  }
  return true;
}

bool isConfiguredMac(const uint8_t *mac) {
  return mac != nullptr && !isPlaceholderMac(mac);
}

const char *controllerNameFromId(uint8_t controllerId) {
  switch (controllerId) {
    case CTRL_LEG:
      return "Leg";
    case CTRL_ROTATING_ARM:
      return "RotatingArm";
    case CTRL_FIXED_STAGE:
      return "FixedStage";
    default:
      return "Unknown";
  }
}

const char *controllerNameFromMac(const uint8_t *mac) {
  if (mac == nullptr) {
    return "Unknown";
  }
  if (memcmp(mac, Leg_Address, 6) == 0) {
    return "Leg";
  }
  if (memcmp(mac, rotatingArm_Address, 6) == 0) {
    return "RotatingArm";
  }
  if (memcmp(mac, fixedStage_Address, 6) == 0) {
    return "FixedStage";
  }
  return "Unknown";
}

void notifyBle(const String &message) {
  Serial.println(message);
  if (bleClientConnected && txCharacteristic != nullptr) {
    txCharacteristic->setValue(message.c_str());
    txCharacteristic->notify();
  }
}

uint8_t normalizeLegSpeed(uint8_t requestedSpeed) {
  return (uint8_t)constrain(requestedSpeed, SPEED_MIN, SPEED_MAX);
}

uint8_t legFullSpeed() {
  return normalizeLegSpeed(currentSpeed);
}

uint8_t legHalfSpeed() {
  return normalizeLegSpeed((uint8_t)max((int)SPEED_MIN, (int)currentSpeed / 2));
}

bool isLegMotionCommand(uint8_t cmd) {
  return cmd == CMD_FORWARD || cmd == CMD_BACKWARD ||
         cmd == CMD_SPIN_LEFT || cmd == CMD_SPIN_RIGHT;
}

String normalizeBleCommandText(String commandText) {
  commandText.trim();
  commandText.toUpperCase();
  commandText.replace(" ", "");
  commandText.replace("_", "");
  commandText.replace("-", "");
  return commandText;
}

String normalizeThetaCommandText(String commandText) {
  commandText.trim();
  commandText.toUpperCase();
  commandText.replace(" ", "");
  commandText.replace("_", "");
  return commandText;
}

void queueLegCommand(uint8_t cmd, uint8_t spd) {
  legCommand = cmd;
  legSpeed = spd;
  legDirty = true;
}

void queueRotatingArmCommand(uint8_t cmd, uint8_t spd) {
  rotatingArmCommand = cmd;
  rotatingArmSpeed = spd;
  rotatingArmDirty = true;
}

void queueFixedStageCommand(uint8_t cmd, uint8_t spd) {
  fixedStageCommand = cmd;
  fixedStageSpeed = spd;
  fixedStageDirty = true;
}

void queueHomeCommand() {
  queueRotatingArmCommand(CMD_ARM_HOME, 0);
  queueFixedStageCommand(CMD_FIXED_GATE_CLOSE, 0);
}

void queueAllStop() {
  queueLegCommand(CMD_LEG_STOP, 0);
  queueRotatingArmCommand(CMD_ARM_STOP, 0);
  queueFixedStageCommand(CMD_FIXED_STOP, 0);
}

bool sendLegCommandNow() {
  if (!isConfiguredMac(Leg_Address)) {
    notifyBle("!! Leg peer MAC not configured");
    return false;
  }

  leg_now_message msg = {legCommand, legSpeed};
  return esp_now_send(Leg_Address, (uint8_t *)&msg, sizeof(msg)) == ESP_OK;
}

bool sendRotatingArmCommandNow() {
  if (!isConfiguredMac(rotatingArm_Address)) {
    notifyBle("!! RotatingArm peer MAC not configured");
    return false;
  }

  rotating_arm_now_message msg = {rotatingArmCommand, rotatingArmSpeed};
  return esp_now_send(rotatingArm_Address, (uint8_t *)&msg, sizeof(msg)) == ESP_OK;
}

bool sendFixedStageCommandNow() {
  if (!isConfiguredMac(fixedStage_Address)) {
    notifyBle("!! FixedStage peer MAC not configured");
    return false;
  }

  fixed_stage_now_message msg = {fixedStageCommand, fixedStageSpeed};
  return esp_now_send(fixedStage_Address, (uint8_t *)&msg, sizeof(msg)) == ESP_OK;
}

void sendPendingCommands() {
  if (legDirty) {
    if (sendLegCommandNow()) {
      lastLegSendMs = millis();
      notifyBle("-> Leg CMD:" + String(legCommand) + " SPD:" + String(legSpeed));
    }
    legDirty = false;
  }

  if (rotatingArmDirty) {
    if (sendRotatingArmCommandNow()) {
      notifyBle("-> RotatingArm CMD:" + String(rotatingArmCommand));
    }
    rotatingArmDirty = false;
  }

  if (fixedStageDirty) {
    if (sendFixedStageCommandNow()) {
      notifyBle("-> FixedStage CMD:" + String(fixedStageCommand));
    }
    fixedStageDirty = false;
  }
}

void sendLegKeepaliveIfNeeded() {
  if (legDirty || !isLegMotionCommand(legCommand)) {
    return;
  }

  if (millis() - lastLegSendMs < COMMAND_KEEPALIVE_INTERVAL_MS) {
    return;
  }

  if (sendLegCommandNow()) {
    lastLegSendMs = millis();
    Serial.println("-> Leg keepalive");
  }
}

// =====================================================
//  ESP-NOW 回調
// =====================================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  const uint8_t *destination = info != nullptr ? info->des_addr : nullptr;
  Serial.print("ESP-NOW -> ");
  Serial.print(controllerNameFromMac(destination));
  Serial.print(": ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (incomingData == nullptr || len != sizeof(ack_message)) {
    Serial.print("Unexpected ESP-NOW response, len=");
    Serial.println(len);
    return;
  }

  ack_message ack;
  memcpy(&ack, incomingData, sizeof(ack));

  const char *sourceName = controllerNameFromId(ack.controller_id);
  if (info != nullptr && info->src_addr != nullptr) {
    const char *macName = controllerNameFromMac(info->src_addr);
    if (strcmp(macName, "Unknown") != 0) {
      sourceName = macName;
    }
  }

  notifyBle(
    String(sourceName) + " ACK cmd=" + String(ack.command) +
    " spd=" + String(ack.speed) +
    " status=" + String(ack.status)
  );
}

// =====================================================
//  指令處理
// =====================================================
void processSingleChar(char c) {
  switch (c) {
    case 'f':
      queueLegCommand(CMD_FORWARD, legFullSpeed());
      notifyBle("CMD: Forward");
      break;
    case 'b':
      queueLegCommand(CMD_BACKWARD, legFullSpeed());
      notifyBle("CMD: Backward");
      break;
    case 'l':
    case 'q':
      queueLegCommand(CMD_SPIN_LEFT, legFullSpeed());
      notifyBle("CMD: SpinLeft");
      break;
    case 'r':
    case 'e':
      queueLegCommand(CMD_SPIN_RIGHT, legFullSpeed());
      notifyBle("CMD: SpinRight");
      break;
    case 'F':
      queueLegCommand(CMD_FORWARD, legHalfSpeed());
      notifyBle("CMD: Forward(half)");
      break;
    case 'B':
      queueLegCommand(CMD_BACKWARD, legHalfSpeed());
      notifyBle("CMD: Backward(half)");
      break;
    case 'L':
      queueLegCommand(CMD_SPIN_LEFT, legHalfSpeed());
      notifyBle("CMD: SpinLeft(half)");
      break;
    case 'R':
      queueLegCommand(CMD_SPIN_RIGHT, legHalfSpeed());
      notifyBle("CMD: SpinRight(half)");
      break;

    case 'a':
      queueFixedStageCommand(CMD_FIXED_SPIN_LEFT, FIXED_SPIN_DEFAULT_SPEED);
      notifyBle("CMD: Fixed Spin Left");
      break;
    case 'd':
      queueFixedStageCommand(CMD_FIXED_SPIN_RIGHT, FIXED_SPIN_DEFAULT_SPEED);
      notifyBle("CMD: Fixed Spin Right");
      break;
    case 'u':
      queueRotatingArmCommand(CMD_ARM_Z_UP, Z_DEFAULT_SPEED);
      notifyBle("CMD: Z Up");
      break;
    case 'j':
      queueRotatingArmCommand(CMD_ARM_Z_DOWN, Z_DEFAULT_SPEED);
      notifyBle("CMD: Z Down");
      break;

    case 'w':
      queueRotatingArmCommand(CMD_ARM_R_EXTEND, 0);
      notifyBle("CMD: r Extend");
      break;
    case 's':
      queueRotatingArmCommand(CMD_ARM_R_RETRACT, 0);
      notifyBle("CMD: r Retract");
      break;
    case 'x':
    case 'X':
      queueRotatingArmCommand(CMD_ARM_R_STOP, 0);
      notifyBle("CMD: r STOP");
      break;
    case 'i':
    case 'I':
      queueRotatingArmCommand(CMD_ARM_THETA_POS, 0);
      notifyBle("CMD: Theta+");
      break;
    case 'k':
    case 'K':
      queueRotatingArmCommand(CMD_ARM_THETA_NEG, 0);
      notifyBle("CMD: Theta-");
      break;
    case 'o':
      queueRotatingArmCommand(CMD_ARM_CLAW_OPEN, 0);
      notifyBle("CMD: Claw Open");
      break;
    case 'p':
      queueRotatingArmCommand(CMD_ARM_CLAW_CLOSE, 0);
      notifyBle("CMD: Claw Close");
      break;
    case 'g':
      queueFixedStageCommand(CMD_FIXED_GATE_OPEN, 0);
      notifyBle("CMD: Gate Open");
      break;
    case 'n':
      queueFixedStageCommand(CMD_FIXED_GATE_CLOSE, 0);
      notifyBle("CMD: Gate Close");
      break;
    case 'h':
      queueHomeCommand();
      notifyBle("CMD: Home");
      break;

    case CMD_LEG_STOP_ONLY:
      queueLegCommand(CMD_LEG_STOP, 0);
      notifyBle("CMD: LEG STOP");
      break;
    case CMD_ARM_STOP_ONLY:
      queueRotatingArmCommand(CMD_ARM_STOP, 0);
      notifyBle("CMD: ARM STOP");
      break;
    case CMD_FIXED_STOP_ONLY:
      queueFixedStageCommand(CMD_FIXED_STOP, 0);
      notifyBle("CMD: FIXED STOP");
      break;
    case '0':
      queueAllStop();
      notifyBle("CMD: ALL STOP");
      break;

    default:
      if (c != '\n' && c != '\r') {
        notifyBle("Unknown cmd: " + String(c));
      }
      break;
  }
}

void handleBleCommand(String commandText) {
  commandText.trim();
  if (commandText.length() == 0) {
    return;
  }

  if (commandText.length() == 1) {
    processSingleChar(commandText.charAt(0));
    return;
  }

  String upperCmd = commandText;
  upperCmd.toUpperCase();
  String normalizedCmd = normalizeBleCommandText(commandText);
  String thetaCmd = normalizeThetaCommandText(commandText);

  if (normalizedCmd == "FORWARD") {
    queueLegCommand(CMD_FORWARD, legFullSpeed());
    notifyBle("CMD: FORWARD");
  } else if (normalizedCmd == "BACKWARD") {
    queueLegCommand(CMD_BACKWARD, legFullSpeed());
    notifyBle("CMD: BACKWARD");
  } else if (normalizedCmd == "LEFT" || normalizedCmd == "QL" || normalizedCmd == "SPINL") {
    queueLegCommand(CMD_SPIN_LEFT, legFullSpeed());
    notifyBle("CMD: SPIN LEFT");
  } else if (normalizedCmd == "RIGHT" || normalizedCmd == "ER" || normalizedCmd == "SPINR") {
    queueLegCommand(CMD_SPIN_RIGHT, legFullSpeed());
    notifyBle("CMD: SPIN RIGHT");
  } else if (normalizedCmd == "LEGSTOP") {
    queueLegCommand(CMD_LEG_STOP, 0);
    notifyBle("CMD: LEG STOP");
  } else if (normalizedCmd == "TL" || normalizedCmd == "TURNTABLELEFT" || normalizedCmd == "FIXEDLEFT") {
    queueFixedStageCommand(CMD_FIXED_SPIN_LEFT, FIXED_SPIN_DEFAULT_SPEED);
    notifyBle("CMD: FIXED SPIN LEFT");
  } else if (normalizedCmd == "TR" || normalizedCmd == "TURNTABLERIGHT" || normalizedCmd == "FIXEDRIGHT") {
    queueFixedStageCommand(CMD_FIXED_SPIN_RIGHT, FIXED_SPIN_DEFAULT_SPEED);
    notifyBle("CMD: FIXED SPIN RIGHT");
  } else if (normalizedCmd == "UP" || normalizedCmd == "ZU") {
    queueRotatingArmCommand(CMD_ARM_Z_UP, Z_DEFAULT_SPEED);
    notifyBle("CMD: Z UP");
  } else if (normalizedCmd == "DOWN" || normalizedCmd == "ZD") {
    queueRotatingArmCommand(CMD_ARM_Z_DOWN, Z_DEFAULT_SPEED);
    notifyBle("CMD: Z DOWN");
  } else if (normalizedCmd == "TZSTOP" || normalizedCmd == "ARMSTOP" || normalizedCmd == "SERVOSTOP") {
    queueRotatingArmCommand(CMD_ARM_STOP, 0);
    notifyBle("CMD: ARM STOP");
  } else if (normalizedCmd == "EXTEND") {
    queueRotatingArmCommand(CMD_ARM_R_EXTEND, 0);
    notifyBle("CMD: R EXTEND");
  } else if (normalizedCmd == "RETRACT") {
    queueRotatingArmCommand(CMD_ARM_R_RETRACT, 0);
    notifyBle("CMD: R RETRACT");
  } else if (normalizedCmd == "RSTOP") {
    queueRotatingArmCommand(CMD_ARM_R_STOP, 0);
    notifyBle("CMD: R STOP");
  } else if (thetaCmd == "THETA+" || thetaCmd == "THETAPLUS" ||
             thetaCmd == "THETAPOS" || thetaCmd == "THETAPOSITIVE" ||
             thetaCmd == "T+") {
    queueRotatingArmCommand(CMD_ARM_THETA_POS, 0);
    notifyBle("CMD: THETA+");
  } else if (thetaCmd == "THETA-" || thetaCmd == "THETAMINUS" ||
             thetaCmd == "THETANEG" || thetaCmd == "THETANEGATIVE" ||
             thetaCmd == "T-") {
    queueRotatingArmCommand(CMD_ARM_THETA_NEG, 0);
    notifyBle("CMD: THETA-");
  } else if (normalizedCmd == "OPEN") {
    queueRotatingArmCommand(CMD_ARM_CLAW_OPEN, 0);
    notifyBle("CMD: CLAW OPEN");
  } else if (normalizedCmd == "CLOSE") {
    queueRotatingArmCommand(CMD_ARM_CLAW_CLOSE, 0);
    notifyBle("CMD: CLAW CLOSE");
  } else if (normalizedCmd == "HOME") {
    queueHomeCommand();
    notifyBle("CMD: HOME");
  } else if (normalizedCmd == "GATEOPEN" || normalizedCmd == "GO") {
    queueFixedStageCommand(CMD_FIXED_GATE_OPEN, 0);
    notifyBle("CMD: GATE OPEN");
  } else if (normalizedCmd == "GATECLOSE" || normalizedCmd == "GC") {
    queueFixedStageCommand(CMD_FIXED_GATE_CLOSE, 0);
    notifyBle("CMD: GATE CLOSE");
  } else if (normalizedCmd == "FIXEDSTOP" || normalizedCmd == "GATESTOP" || normalizedCmd == "TSTOP") {
    queueFixedStageCommand(CMD_FIXED_STOP, 0);
    notifyBle("CMD: FIXED STOP");
  } else if (normalizedCmd == "STOP" || normalizedCmd == "ALLSTOP") {
    queueAllStop();
    notifyBle("CMD: ALL STOP");
  } else if (upperCmd.startsWith("SPD:") || upperCmd.startsWith("SPEED:")) {
    int separatorIndex = upperCmd.indexOf(':');
    int parsedSpeed = upperCmd.substring(separatorIndex + 1).toInt();
    currentSpeed = normalizeLegSpeed((uint8_t)constrain(parsedSpeed, 0, 255));
    notifyBle("Speed set to " + String(currentSpeed) + " (leg default)");
  } else {
    notifyBle("Unknown BLE cmd: " + commandText);
  }
}

// =====================================================
//  BLE Callbacks
// =====================================================
class ControllerServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleClientConnected = true;
    notifyBle("BLE controller connected");
  }

  void onDisconnect(BLEServer *server) override {
    bleClientConnected = false;
    queueAllStop();
    server->getAdvertising()->start();
    Serial.println("BLE disconnected, advertising restarted, ALL STOP queued");
  }
};

class ControllerRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String commandText = characteristic->getValue();
    if (commandText.length() == 0) {
      return;
    }
    handleBleCommand(commandText);
  }
};

// =====================================================
//  ESP-NOW 初始化
// =====================================================
bool addPeer(uint8_t *addr, const char *name) {
  if (!isConfiguredMac(addr)) {
    Serial.print("Peer skipped (placeholder MAC): ");
    Serial.println(name);
    return false;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, addr, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.print("Failed to add peer: ");
    Serial.println(name);
    return false;
  }

  Serial.print("Peer added: ");
  Serial.println(name);
  return true;
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

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  addPeer(Leg_Address, "C3#1 Leg");
  addPeer(rotatingArm_Address, "C3#2 RotatingArm");
  addPeer(fixedStage_Address, "C3#3 FixedStage");

  Serial.println("ESP-NOW initialized");
}

// =====================================================
//  BLE 初始化
// =====================================================
void setupBle() {
  BLEDevice::init(BLE_DEVICE_NAME);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ControllerServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  txCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *rxCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  rxCharacteristic->setCallbacks(new ControllerRxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("BLE ready");
  Serial.println("Device name: " + String(BLE_DEVICE_NAME));
}

void printMacConfigWarning(const char *name, const uint8_t *mac) {
  if (!isConfiguredMac(mac)) {
    Serial.print("!! WARNING: ");
    Serial.print(name);
    Serial.println(" MAC is still placeholder FF:FF:FF:FF:FF:FF");
  }
}

// =====================================================
//  Setup / Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("========================================");
  Serial.println("  ESP32 MainController — BLE Version");
  Serial.println("  BLE Controller – Arduino ESP32");
  Serial.println("========================================");

  Serial.print("Main MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Leg peer: ");
  printMacAddress(Leg_Address);
  Serial.println();
  Serial.print("RotatingArm peer: ");
  printMacAddress(rotatingArm_Address);
  Serial.println();
  Serial.print("FixedStage peer: ");
  printMacAddress(fixedStage_Address);
  Serial.println();

  printMacConfigWarning("RotatingArm", rotatingArm_Address);
  printMacConfigWarning("FixedStage", fixedStage_Address);

  setupEspNow();
  setupBle();
  queueAllStop();

  Serial.println();
  Serial.println("--- Command Reference ---");
  Serial.println(" Leg move:   f/b/l/r/q/e  | half: F/B/L/R");
  Serial.println(" Fixed:      a=spin left d=spin right  g=gate open n=gate close");
  Serial.println(" Arm:        u=z up j=z down  w/s/x/i/k/o/p/h");
  Serial.println(" Split stop: 1=LEGSTOP 2=ARMSTOP/TZSTOP/SERVOSTOP 3=FIXEDSTOP");
  Serial.println(" All stop:   0=STOP ALL");
  Serial.println(" BLE text:   FORWARD BACKWARD LEFT RIGHT LEGSTOP");
  Serial.println("             TL TR FIXEDSTOP");
  Serial.println("             UP DOWN ARMSTOP TZSTOP SERVOSTOP");
  Serial.println("             EXTEND RETRACT RSTOP THETA+ THETA- OPEN CLOSE HOME");
  Serial.println("             GATEOPEN GATECLOSE STOP");
  Serial.println("             SPD:150");
  Serial.println("-------------------------");
}

void loop() {
  if (!bleClientConnected && lastBleClientConnected) {
    Serial.println("BLE controller disconnected");
  }

  if (bleClientConnected && !lastBleClientConnected) {
    Serial.println("BLE controller ready");
  }

  lastBleClientConnected = bleClientConnected;

  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("[Serial] ");
    Serial.println(c);
    processSingleChar(c);
  }

  sendPendingCommands();
  sendLegKeepaliveIfNeeded();
  delay(20);
}
