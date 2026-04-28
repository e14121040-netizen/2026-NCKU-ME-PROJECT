/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 主控板
 *  ESP32 MainController (BLE 版)
 * =====================================================
 *
 *  功能：
 *  - 透過 BLE (低功耗藍芽) UART Service 接收手機遙控 App 指令
 *  - 透過 ESP-NOW 將指令分發給 3 塊 ESP32-C3 子控板
 *  - 支援指令字串和單字元兩種格式
 *  - 支援 ACK 回傳（從子控板接收確認）
 *  - 不直接驅動任何馬達
 *
 *  分散式架構：
 *  ┌─────────────────────────────────────────────┐
 *  │  ESP32 主控板 (BLE)                          │
 *  │  ├── BLE UART Service 接收                  │
 *  │  └── ESP-NOW 分發至：                        │
 *  │       ├── C3 #1 (腿部)  → 步行馬達 ×2       │
 *  │       ├── C3 #2 (手臂)  → r/θ 馬達 ×2       │
 *  │       └── C3 #3 (z/夾爪) → z馬達+伺服       │
 *  └─────────────────────────────────────────────┘
 *
 *  BLE 指令格式：
 *  字串指令（大寫）：
 *    "F" / "FORWARD"  = 前進     "B" / "BACKWARD" = 後退
 *    "L" / "LEFT"     = 左轉     "R" / "RIGHT"    = 右轉
 *    "QL" / "SPINL"   = 左旋     "ER" / "SPINR"   = 右旋
 *    "S" / "STOP"     = 全部停止
 *    "SPD:xxx"        = 設定速度 (0~255)
 *
 *  單字元指令（與 BluetoothSerial 版相容）：
 *    腿部: f/b/l/r/q/e/F/B/L/R
 *    手臂: w/s/a/d
 *    z/夾爪: u/j/o/p/t/y/h
 *    停止: 0
 *
 *  BLE UART Service UUID:
 *    Service:  6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *    RX Char:  6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 *    TX Char:  6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

const uint8_t ESPNOW_CHANNEL = 1;

// =====================================================
//  子控板 MAC 地址
//  ⚠ TODO: 將 0xFF 佔位符替換為實際 MAC 地址！
//  取得方式：上傳 macaddress/macaddress.ino 至各 C3，
//           開啟 Serial Monitor (115200) 記錄 MAC。
// =====================================================
uint8_t Leg_Address[]     = {0x58, 0x8C, 0x81, 0x9D, 0xF6, 0x90};  // C3 #1 腿部
uint8_t r_theta_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // C3 #2 手臂 r/θ  ⚠ TODO
uint8_t z_clap_Address[]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // C3 #3 z/夾爪    ⚠ TODO

// =====================================================
//  BLE UART Service UUID
// =====================================================
static const char *BLE_DEVICE_NAME = "ESP32_MainController";
static const char *SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *CHARACTERISTIC_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *CHARACTERISTIC_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// =====================================================
//  ESP-NOW 資料格式 — 通用指令結構
// =====================================================
// 腿部指令
enum LegCommand {
  CMD_LEG_STOP = 0,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_LEFT,
  CMD_RIGHT,
  CMD_SPIN_LEFT,
  CMD_SPIN_RIGHT,
};

typedef struct leg_now_message {
  uint8_t command;
  uint8_t speed;
} leg_now_message;

typedef struct leg_ack_message {
  uint8_t command;
  uint8_t speed;
  uint8_t status;
} leg_ack_message;

// 手臂 r/θ 指令
enum GripperCommand {
  CMD_GRIPPER_STOP = 0,
  CMD_ARM_EXTEND,    // r+
  CMD_ARM_RETRACT,   // r-
  CMD_ARM_LEFT,      // θ-
  CMD_ARM_RIGHT,     // θ+
};

typedef struct gripper_now_message {
  uint8_t command;
  uint8_t speed;
} gripper_now_message;

// z/夾爪指令
enum ZClawCommand {
  CMD_ZCLAW_STOP = 0,
  CMD_Z_UP,          // z 上升
  CMD_Z_DOWN,        // z 下降
  CMD_CLAW_OPEN,     // 夾爪張開（伺服）
  CMD_CLAW_CLOSE,    // 夾爪閉合（伺服）
  CMD_LID_OPEN,      // 蓋板開（伺服）
  CMD_LID_CLOSE,     // 蓋板關（伺服）
  CMD_HOME,          // 歸位
};

typedef struct zclaw_now_message {
  uint8_t command;
  uint8_t speed;     // DC 馬達速度 or 伺服角度
} zclaw_now_message;

// =====================================================
//  速度設定
// =====================================================
const uint8_t FULL_SPEED = 200;
const uint8_t HALF_SPEED = 120;
uint8_t currentSpeed = FULL_SPEED;  // BLE SPD 指令可動態調整

// =====================================================
//  BLE 相關變數
// =====================================================
BLECharacteristic *txCharacteristic = nullptr;
bool bleClientConnected = false;
bool lastBleClientConnected = false;

// =====================================================
//  指令佇列
// =====================================================
// 腿部指令佇列
uint8_t legCommand = CMD_LEG_STOP;
uint8_t legSpeed = FULL_SPEED;
bool legDirty = false;

// 手臂指令佇列
uint8_t gripperCommand = CMD_GRIPPER_STOP;
uint8_t gripperSpeed = FULL_SPEED;
bool gripperDirty = false;

// z/夾爪指令佇列
uint8_t zclawCommand = CMD_ZCLAW_STOP;
uint8_t zclawSpeed = FULL_SPEED;
bool zclawDirty = false;

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

void notifyBle(const String &message) {
  Serial.println(message);
  if (bleClientConnected && txCharacteristic != nullptr) {
    txCharacteristic->setValue(message.c_str());
    txCharacteristic->notify();
  }
}

// =====================================================
//  指令佇列操作
// =====================================================
void queueLegCommand(uint8_t cmd, uint8_t spd) {
  legCommand = cmd;
  legSpeed = spd;
  legDirty = true;
}

void queueGripperCommand(uint8_t cmd, uint8_t spd) {
  gripperCommand = cmd;
  gripperSpeed = spd;
  gripperDirty = true;
}

void queueZClawCommand(uint8_t cmd, uint8_t spd) {
  zclawCommand = cmd;
  zclawSpeed = spd;
  zclawDirty = true;
}

// =====================================================
//  指令發送（定時從佇列發送）
// =====================================================
void sendPendingCommands() {
  if (legDirty) {
    leg_now_message msg = {legCommand, legSpeed};
    esp_err_t result = esp_now_send(Leg_Address, (uint8_t *)&msg, sizeof(msg));
    if (result == ESP_OK) {
      notifyBle("-> Leg CMD:" + String(legCommand) + " SPD:" + String(legSpeed));
    } else {
      notifyBle("!! Leg send err:" + String(result));
    }
    legDirty = false;
  }

  if (gripperDirty) {
    gripper_now_message msg = {gripperCommand, gripperSpeed};
    esp_err_t result = esp_now_send(r_theta_Address, (uint8_t *)&msg, sizeof(msg));
    if (result == ESP_OK) {
      notifyBle("-> Gripper CMD:" + String(gripperCommand) + " SPD:" + String(gripperSpeed));
    } else {
      notifyBle("!! Gripper send err:" + String(result));
    }
    gripperDirty = false;
  }

  if (zclawDirty) {
    zclaw_now_message msg = {zclawCommand, zclawSpeed};
    esp_err_t result = esp_now_send(z_clap_Address, (uint8_t *)&msg, sizeof(msg));
    if (result == ESP_OK) {
      notifyBle("-> ZClaw CMD:" + String(zclawCommand) + " SPD:" + String(zclawSpeed));
    } else {
      notifyBle("!! ZClaw send err:" + String(result));
    }
    zclawDirty = false;
  }
}

// =====================================================
//  ESP-NOW 回調
// =====================================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  const uint8_t *destination = info != nullptr ? info->des_addr : nullptr;

  if (destination != nullptr && memcmp(destination, Leg_Address, 6) == 0) {
    Serial.print("ESP-NOW -> Leg: ");
  } else if (destination != nullptr && memcmp(destination, r_theta_Address, 6) == 0) {
    Serial.print("ESP-NOW -> R_Theta: ");
  } else if (destination != nullptr && memcmp(destination, z_clap_Address, 6) == 0) {
    Serial.print("ESP-NOW -> Z_Claw: ");
  } else {
    Serial.print("ESP-NOW -> Unknown: ");
  }
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (incomingData == nullptr || len < 2) {
    Serial.print("Unexpected ESP-NOW response, len=");
    Serial.println(len);
    return;
  }

  // 嘗試解析為 ACK 回應
  if (info != nullptr && info->src_addr != nullptr) {
    if (memcmp(info->src_addr, Leg_Address, 6) == 0 && len == sizeof(leg_ack_message)) {
      leg_ack_message ack;
      memcpy(&ack, incomingData, sizeof(ack));
      notifyBle("Leg ACK cmd=" + String(ack.command) +
                ", spd=" + String(ack.speed) +
                ", status=" + String(ack.status));
    } else {
      Serial.print("ACK from unknown peer, len=");
      Serial.println(len);
    }
  }
}

// =====================================================
//  處理單字元指令（與 BluetoothSerial 版相容）
// =====================================================
void processSingleChar(char c) {
  switch (c) {
    // ===== 腿部指令 → C3 #1 =====
    case 'f': queueLegCommand(CMD_FORWARD, FULL_SPEED); notifyBle("CMD: Forward"); break;
    case 'b': queueLegCommand(CMD_BACKWARD, FULL_SPEED); notifyBle("CMD: Backward"); break;
    case 'l': queueLegCommand(CMD_LEFT, FULL_SPEED); notifyBle("CMD: Left"); break;
    case 'r': queueLegCommand(CMD_RIGHT, FULL_SPEED); notifyBle("CMD: Right"); break;
    case 'q': queueLegCommand(CMD_SPIN_LEFT, FULL_SPEED); notifyBle("CMD: SpinLeft"); break;
    case 'e': queueLegCommand(CMD_SPIN_RIGHT, FULL_SPEED); notifyBle("CMD: SpinRight"); break;
    case 'F': queueLegCommand(CMD_FORWARD, HALF_SPEED); notifyBle("CMD: Forward(half)"); break;
    case 'B': queueLegCommand(CMD_BACKWARD, HALF_SPEED); notifyBle("CMD: Backward(half)"); break;
    case 'L': queueLegCommand(CMD_LEFT, HALF_SPEED); notifyBle("CMD: Left(half)"); break;
    case 'R': queueLegCommand(CMD_RIGHT, HALF_SPEED); notifyBle("CMD: Right(half)"); break;

    // ===== 手臂 r/θ → C3 #2 =====
    case 'w': queueGripperCommand(CMD_ARM_EXTEND, FULL_SPEED); notifyBle("CMD: Arm Extend"); break;
    case 's': queueGripperCommand(CMD_ARM_RETRACT, FULL_SPEED); notifyBle("CMD: Arm Retract"); break;
    case 'a': queueGripperCommand(CMD_ARM_LEFT, FULL_SPEED); notifyBle("CMD: Arm Left"); break;
    case 'd': queueGripperCommand(CMD_ARM_RIGHT, FULL_SPEED); notifyBle("CMD: Arm Right"); break;

    // ===== z/夾爪 → C3 #3 =====
    case 'u': queueZClawCommand(CMD_Z_UP, FULL_SPEED); notifyBle("CMD: Z Up"); break;
    case 'j': queueZClawCommand(CMD_Z_DOWN, FULL_SPEED); notifyBle("CMD: Z Down"); break;
    case 'o': queueZClawCommand(CMD_CLAW_OPEN, 0); notifyBle("CMD: Claw Open"); break;
    case 'p': queueZClawCommand(CMD_CLAW_CLOSE, 0); notifyBle("CMD: Claw Close"); break;
    case 't': queueZClawCommand(CMD_LID_OPEN, 0); notifyBle("CMD: Lid Open"); break;
    case 'y': queueZClawCommand(CMD_LID_CLOSE, 0); notifyBle("CMD: Lid Close"); break;
    case 'h': queueZClawCommand(CMD_HOME, 0); notifyBle("CMD: Home"); break;

    // ===== 全部停止 =====
    case '0':
      queueLegCommand(CMD_LEG_STOP, 0);
      queueGripperCommand(CMD_GRIPPER_STOP, 0);
      queueZClawCommand(CMD_ZCLAW_STOP, 0);
      notifyBle("CMD: ALL STOP");
      break;

    default:
      if (c != '\n' && c != '\r') {
        notifyBle("Unknown cmd: " + String(c));
      }
      break;
  }
}

// =====================================================
//  處理 BLE 字串指令
// =====================================================
void handleBleCommand(String commandText) {
  commandText.trim();

  if (commandText.length() == 0) {
    return;
  }

  // 單字元指令直接處理
  if (commandText.length() == 1) {
    processSingleChar(commandText.charAt(0));
    return;
  }

  // 多字元字串指令（轉大寫比較）
  String upperCmd = commandText;
  upperCmd.toUpperCase();

  // ----- 腿部字串指令 -----
  if (upperCmd == "FORWARD") {
    queueLegCommand(CMD_FORWARD, FULL_SPEED);
    notifyBle("CMD: FORWARD");
  } else if (upperCmd == "BACKWARD") {
    queueLegCommand(CMD_BACKWARD, FULL_SPEED);
    notifyBle("CMD: BACKWARD");
  } else if (upperCmd == "LEFT") {
    queueLegCommand(CMD_LEFT, FULL_SPEED);
    notifyBle("CMD: LEFT");
  } else if (upperCmd == "RIGHT") {
    queueLegCommand(CMD_RIGHT, FULL_SPEED);
    notifyBle("CMD: RIGHT");
  } else if (upperCmd == "QL" || upperCmd == "SPINL") {
    queueLegCommand(CMD_SPIN_LEFT, FULL_SPEED);
    notifyBle("CMD: SPIN LEFT");
  } else if (upperCmd == "ER" || upperCmd == "SPINR") {
    queueLegCommand(CMD_SPIN_RIGHT, FULL_SPEED);
    notifyBle("CMD: SPIN RIGHT");
  }
  // ----- 手臂字串指令 -----
  else if (upperCmd == "EXTEND") {
    queueGripperCommand(CMD_ARM_EXTEND, FULL_SPEED);
    notifyBle("CMD: ARM EXTEND");
  } else if (upperCmd == "RETRACT") {
    queueGripperCommand(CMD_ARM_RETRACT, FULL_SPEED);
    notifyBle("CMD: ARM RETRACT");
  } else if (upperCmd == "ARMLEFT" || upperCmd == "AL") {
    queueGripperCommand(CMD_ARM_LEFT, FULL_SPEED);
    notifyBle("CMD: ARM LEFT");
  } else if (upperCmd == "ARMRIGHT" || upperCmd == "AR") {
    queueGripperCommand(CMD_ARM_RIGHT, FULL_SPEED);
    notifyBle("CMD: ARM RIGHT");
  }
  // ----- z/夾爪字串指令 -----
  else if (upperCmd == "UP" || upperCmd == "ZU") {
    queueZClawCommand(CMD_Z_UP, FULL_SPEED);
    notifyBle("CMD: Z UP");
  } else if (upperCmd == "DOWN" || upperCmd == "ZD") {
    queueZClawCommand(CMD_Z_DOWN, FULL_SPEED);
    notifyBle("CMD: Z DOWN");
  } else if (upperCmd == "OPEN") {
    queueZClawCommand(CMD_CLAW_OPEN, 0);
    notifyBle("CMD: CLAW OPEN");
  } else if (upperCmd == "CLOSE") {
    queueZClawCommand(CMD_CLAW_CLOSE, 0);
    notifyBle("CMD: CLAW CLOSE");
  } else if (upperCmd == "TILT" || upperCmd == "LO") {
    queueZClawCommand(CMD_LID_OPEN, 0);
    notifyBle("CMD: LID OPEN");
  } else if (upperCmd == "FLAT" || upperCmd == "LC") {
    queueZClawCommand(CMD_LID_CLOSE, 0);
    notifyBle("CMD: LID CLOSE");
  } else if (upperCmd == "HOME") {
    queueZClawCommand(CMD_HOME, 0);
    notifyBle("CMD: HOME");
  }
  // ----- 全部停止 -----
  else if (upperCmd == "STOP") {
    queueLegCommand(CMD_LEG_STOP, 0);
    queueGripperCommand(CMD_GRIPPER_STOP, 0);
    queueZClawCommand(CMD_ZCLAW_STOP, 0);
    notifyBle("CMD: ALL STOP");
  }
  // ----- 速度設定 -----
  else if (upperCmd.startsWith("SPD:") || upperCmd.startsWith("SPEED:")) {
    int separatorIndex = upperCmd.indexOf(':');
    int parsedSpeed = upperCmd.substring(separatorIndex + 1).toInt();
    parsedSpeed = constrain(parsedSpeed, 0, 255);
    currentSpeed = (uint8_t)parsedSpeed;
    notifyBle("Speed set to " + String(currentSpeed));
  }
  // ----- 未知指令 -----
  else {
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
    // 斷線時急停所有子控板
    queueLegCommand(CMD_LEG_STOP, 0);
    queueGripperCommand(CMD_GRIPPER_STOP, 0);
    queueZClawCommand(CMD_ZCLAW_STOP, 0);
    server->getAdvertising()->start();
    Serial.println("BLE disconnected, advertising restarted, ALL STOP");
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

  // 註冊 3 個子控板 Peer
  addPeer(Leg_Address, "C3#1 Leg");
  addPeer(r_theta_Address, "C3#2 R_Theta");
  addPeer(z_clap_Address, "C3#3 Z_Claw");

  Serial.println("ESP-NOW initialized with 3 peers");
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

// =====================================================
//  Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("========================================");
  Serial.println("  ESP32 MainController — BLE Version");
  Serial.println("  BLE UART + ESP-NOW Dispatch (3 peers)");
  Serial.println("========================================");

  Serial.print("Main MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Leg peer: ");
  printMacAddress(Leg_Address);
  Serial.println();
  Serial.print("Gripper peer: ");
  printMacAddress(r_theta_Address);
  Serial.println();
  Serial.print("ZClaw peer: ");
  printMacAddress(z_clap_Address);
  Serial.println();

  setupEspNow();
  setupBle();

  // 初始化：全部停止
  queueLegCommand(CMD_LEG_STOP, 0);
  queueGripperCommand(CMD_GRIPPER_STOP, 0);
  queueZClawCommand(CMD_ZCLAW_STOP, 0);

  Serial.println();
  Serial.println("--- Command Reference ---");
  Serial.println("  Leg:     f/b/l/r/q/e (full)  F/B/L/R (half)");
  Serial.println("  Arm:     w=extend s=retract a=left d=right");
  Serial.println("  Z/Claw:  u=up j=down o=open p=close");
  Serial.println("           t=lid_open y=lid_close h=home");
  Serial.println("  All:     0=STOP ALL");
  Serial.println("  BLE:     FORWARD, BACKWARD, LEFT, RIGHT, STOP, SPD:200");
  Serial.println("-------------------------");
}

// =====================================================
//  Main Loop
// =====================================================
void loop() {
  // ----- BLE 連線狀態追蹤 -----
  if (!bleClientConnected && lastBleClientConnected) {
    notifyBle("BLE controller disconnected");
  }

  if (bleClientConnected && !lastBleClientConnected) {
    notifyBle("BLE controller ready");
  }

  lastBleClientConnected = bleClientConnected;

  // ----- Serial 手動測試（debug 用）-----
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("[Serial] ");
    processSingleChar(c);
  }

  // ----- 發送待處理指令 -----
  sendPendingCommands();

  delay(20);
}
