/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 主控板
 *  ESP32 MainController
 * =====================================================
 *
 *  功能：
 *  - 透過藍芽 (BluetoothSerial) 接收手機遙控 App 指令
 *  - 透過 ESP-NOW 將指令分發給 3 塊 ESP32-C3 子控板
 *  - 不直接驅動任何馬達
 *
 *  分散式架構：
 *  ┌─────────────────────────────────────────────┐
 *  │  ESP32 主控板                                │
 *  │  ├── 藍芽接收 (BluetoothSerial)             │
 *  │  └── ESP-NOW 分發至：                        │
 *  │       ├── C3 #1 (腿部)  → 步行馬達 ×2       │
 *  │       ├── C3 #2 (手臂)  → r/θ 馬達 ×2       │
 *  │       └── C3 #3 (z/夾爪) → z馬達+伺服       │
 *  └─────────────────────────────────────────────┘
 *
 *  藍芽指令協議（單字元）：
 *  腿部指令 → 轉發至 C3 #1：
 *    'f' = 前進    'b' = 後退    'l' = 左轉    'r' = 右轉
 *    'q' = 左旋    'e' = 右旋    '0' = 停止
 *    'F' = 半速前進 'B' = 半速後退 'L' = 半速左轉 'R' = 半速右轉
 *
 *  手臂 r/θ 指令 → 轉發至 C3 #2：
 *    'w' = 手臂伸出 (r+)   's' = 手臂縮回 (r-)
 *    'a' = 手臂左轉 (θ-)   'd' = 手臂右轉 (θ+)
 *
 *  z/夾爪指令 → 轉發至 C3 #3：
 *    'u' = z 上升    'j' = z 下降
 *    'o' = 爪張開    'p' = 爪閉合
 *    't' = 承物台傾斜 'y' = 承物台水平
 *    'h' = 歸位
 */

#include <esp_now.h>
#include <WiFi.h>
<<<<<<< HEAD
#include <esp_wifi.h>
#include <BLEDevice.h> // BLE函式庫，提供 BLE 功能
#include <BLEServer.h> // BLE 伺服器相關功能
#include <BLEUtils.h> // BLE 工具函式庫
#include <BLE2902.h>  // BLE 描述符，允許客戶端訂閱通知

const uint8_t ESPNOW_CHANNEL = 1;

// 接收端 MAC 地址
uint8_t Leg_Address[] = {0x58, 0x8C, 0x81, 0x9D, 0xF6, 0x90};
uint8_t r_theta_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t z_clap_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// BLE UART Service UUID，很多 BLE Controller App 可直接使用
static const char *BLE_DEVICE_NAME = "ESP32_MainController";
//UUID 常用，BLE controller app 也可直接使用這組 UUID 來連接和傳輸資料
static const char *SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *CHARACTERISTIC_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *CHARACTERISTIC_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// 腿部機構指令列舉
enum Leg_CommandType {
  CMD_STOP = 0,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_LEFT,
  CMD_RIGHT
};

// ESP-NOW 腿部傳輸資料格式，uint8_t 節省傳輸空間，命令和速度都限制在 0-255 範圍內
typedef struct Leg_now_message {
  uint8_t command;
  uint8_t speed;
} Leg_now_message;

typedef struct Leg_ack_message {
  uint8_t command;
  uint8_t speed;
  uint8_t status;
} Leg_ack_message;

Leg_now_message leg_msg = {CMD_STOP, 180};//常態限制在 180，避免過快導致機構損壞

// --------BLE 相關變數---------
BLECharacteristic *txCharacteristic = nullptr;
bool bleClientConnected = false;
bool lastBleClientConnected = false;

uint8_t currentCommand = CMD_STOP;
uint8_t currentSpeed = 180;
bool commandDirty = false;

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

void queueLegCommand(uint8_t command, uint8_t speed) {
  currentCommand = command;
  currentSpeed = speed;
  commandDirty = true;
}

void sendLegCommandIfNeeded() {
  if (!commandDirty) {
    return;
  }

  leg_msg.command = currentCommand;
  leg_msg.speed = currentSpeed;

  esp_err_t result = esp_now_send(Leg_Address, (uint8_t *)&leg_msg, sizeof(leg_msg));
  if (result == ESP_OK) {
    notifyBle("Queued ESP-NOW command: " + String(currentCommand) + ", speed=" + String(currentSpeed));
  } else {
    notifyBle("ESP-NOW send error: " + String(result));
  }

  commandDirty = false;
}

// 資料送出後，系統自動呼叫函式，告訴這次傳輸成功或失敗
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  const uint8_t *destination = info != nullptr ? info->des_addr : nullptr;

  //腿部連線回報
  if (destination != nullptr && memcmp(destination, Leg_Address, 6) == 0) {
    Serial.print("Last Packet Sent to: ");
    Serial.println("Leg");
  } 
  //R_Theta 連線回報
  else if (destination != nullptr && memcmp(destination, r_theta_Address, 6) == 0) {
    Serial.print("Last Packet Sent to: ");
    Serial.println("R_Theta");
  } 
  //Z_Clap 連線回報
  else if (destination != nullptr && memcmp(destination, z_clap_Address, 6) == 0) {
    Serial.print("Last Packet Sent to: ");
    Serial.println("Z_Clap");
=======
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// =====================================================
//  子控板 MAC 地址（需替換為實際 MAC）
// =====================================================
uint8_t Leg_Address[]     = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // C3 #1 腿部
uint8_t r_theta_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // C3 #2 手臂 r/θ
uint8_t z_clap_Address[]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // C3 #3 z/夾爪

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
  CMD_PLATFORM_TILT, // 承物台傾斜（伺服）
  CMD_PLATFORM_FLAT, // 承物台水平（伺服）
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

// =====================================================
//  ESP-NOW 回調：傳送結果
// =====================================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send -> ");
  if (memcmp(mac_addr, Leg_Address, 6) == 0) {
    Serial.print("Leg");
  } else if (memcmp(mac_addr, r_theta_Address, 6) == 0) {
    Serial.print("R_Theta");
  } else if (memcmp(mac_addr, z_clap_Address, 6) == 0) {
    Serial.print("Z_Claw");
>>>>>>> e993a25612e77d4dc14e1e2119fa60f8b33a3675
  } else {
    Serial.print("Unknown");
  }
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " OK" : " FAIL");
}

<<<<<<< HEAD
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (incomingData == nullptr || len != sizeof(Leg_ack_message)) {
    Serial.print("Unexpected ESP-NOW response, len=");
    Serial.println(len);
    return;
  }

  if (info == nullptr || info->src_addr == nullptr || memcmp(info->src_addr, Leg_Address, 6) != 0) {
    Serial.println("Ignored ESP-NOW response from unknown peer");
    return;
  }

  Leg_ack_message ack;
  memcpy(&ack, incomingData, sizeof(ack));

  String message = "Leg ACK cmd=" + String(ack.command) +
                   ", speed=" + String(ack.speed) +
                   ", status=" + String(ack.status);
  notifyBle(message);
}


class ControllerServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleClientConnected = true;
    notifyBle("BLE controller connected");
  }

  void onDisconnect(BLEServer *server) override {
    bleClientConnected = false;
    queueLegCommand(CMD_STOP, currentSpeed);
    server->getAdvertising()->start();
    Serial.println("BLE controller disconnected, advertising restarted");
  }
};

void handleBleCommand(String commandText) {
  commandText.trim();
  commandText.toUpperCase();

  if (commandText.length() == 0) {
    return;
  }

  if (commandText == "F" || commandText == "FORWARD") {
    queueLegCommand(CMD_FORWARD, currentSpeed);
    notifyBle("Command: FORWARD");
    return;
  }

  if (commandText == "B" || commandText == "BACKWARD") {
    queueLegCommand(CMD_BACKWARD, currentSpeed);
    notifyBle("Command: BACKWARD");
    return;
  }

  if (commandText == "L" || commandText == "LEFT") {
    queueLegCommand(CMD_LEFT, currentSpeed);
    notifyBle("Command: LEFT");
    return;
  }

  if (commandText == "R" || commandText == "RIGHT") {
    queueLegCommand(CMD_RIGHT, currentSpeed);
    notifyBle("Command: RIGHT");
    return;
  }

  if (commandText == "S" || commandText == "STOP") {
    queueLegCommand(CMD_STOP, currentSpeed);
    notifyBle("Command: STOP");
    return;
  }

  if (commandText.startsWith("SPD:") || commandText.startsWith("SPEED:")) {
    int separatorIndex = commandText.indexOf(':');
    int parsedSpeed = commandText.substring(separatorIndex + 1).toInt();
    parsedSpeed = constrain(parsedSpeed, 0, 255);
    currentSpeed = (uint8_t)parsedSpeed;
    notifyBle("Speed set to " + String(currentSpeed));
    return;
  }

  notifyBle("Unknown BLE command: " + commandText);
}

class ControllerRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String commandText = characteristic->getValue();
    if (commandText.length() == 0) {
      return;
    }
    handleBleCommand(commandText);
  }
};

//========ESP-NOW 初始化========
void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t channelResult = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channelResult != ESP_OK) {
    Serial.print("Failed to set WiFi channel, err=");
    Serial.println(channelResult);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, Leg_Address, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW initialized");
}

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
  Serial.println("Device name: ESP32_MainController");
  Serial.println("BLE command examples: F, B, L, R, S, SPD:200");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.print("Main MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Leg peer MAC: ");
  printMacAddress(Leg_Address);
  Serial.println();
  setupEspNow();
  setupBle();
  queueLegCommand(CMD_STOP, currentSpeed);
}

void loop() {
  if (!bleClientConnected && lastBleClientConnected) {
    notifyBle("BLE controller disconnected");
  }

  if (bleClientConnected && !lastBleClientConnected) {
    notifyBle("BLE controller ready");
  }

  lastBleClientConnected = bleClientConnected;
  sendLegCommandIfNeeded();
  delay(20);
=======
// =====================================================
//  指令發送函式
// =====================================================
void sendLegCommand(uint8_t cmd, uint8_t spd) {
  leg_now_message msg;
  msg.command = cmd;
  msg.speed = spd;
  esp_now_send(Leg_Address, (uint8_t *)&msg, sizeof(msg));
  Serial.print("-> Leg CMD: "); Serial.print(cmd);
  Serial.print(", SPD: "); Serial.println(spd);
}

void sendGripperCommand(uint8_t cmd, uint8_t spd) {
  gripper_now_message msg;
  msg.command = cmd;
  msg.speed = spd;
  esp_now_send(r_theta_Address, (uint8_t *)&msg, sizeof(msg));
  Serial.print("-> Gripper CMD: "); Serial.print(cmd);
  Serial.print(", SPD: "); Serial.println(spd);
}

void sendZClawCommand(uint8_t cmd, uint8_t spd) {
  zclaw_now_message msg;
  msg.command = cmd;
  msg.speed = spd;
  esp_now_send(z_clap_Address, (uint8_t *)&msg, sizeof(msg));
  Serial.print("-> ZClaw CMD: "); Serial.print(cmd);
  Serial.print(", SPD: "); Serial.println(spd);
}

// =====================================================
//  ESP-NOW Peer 註冊
// =====================================================
bool addPeer(uint8_t *addr, const char *name) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, addr, 6);
  peerInfo.channel = 0;
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

// =====================================================
//  處理藍芽收到的指令
// =====================================================
void processCommand(char c) {
  switch (c) {
    // ===== 腿部指令 → C3 #1 =====
    case 'f': sendLegCommand(CMD_FORWARD, FULL_SPEED); break;
    case 'b': sendLegCommand(CMD_BACKWARD, FULL_SPEED); break;
    case 'l': sendLegCommand(CMD_LEFT, FULL_SPEED); break;
    case 'r': sendLegCommand(CMD_RIGHT, FULL_SPEED); break;
    case 'q': sendLegCommand(CMD_SPIN_LEFT, FULL_SPEED); break;
    case 'e': sendLegCommand(CMD_SPIN_RIGHT, FULL_SPEED); break;
    case 'F': sendLegCommand(CMD_FORWARD, HALF_SPEED); break;
    case 'B': sendLegCommand(CMD_BACKWARD, HALF_SPEED); break;
    case 'L': sendLegCommand(CMD_LEFT, HALF_SPEED); break;
    case 'R': sendLegCommand(CMD_RIGHT, HALF_SPEED); break;

    // ===== 手臂 r/θ → C3 #2 =====
    case 'w': sendGripperCommand(CMD_ARM_EXTEND, FULL_SPEED); break;
    case 's': sendGripperCommand(CMD_ARM_RETRACT, FULL_SPEED); break;
    case 'a': sendGripperCommand(CMD_ARM_LEFT, FULL_SPEED); break;
    case 'd': sendGripperCommand(CMD_ARM_RIGHT, FULL_SPEED); break;

    // ===== z/夾爪 → C3 #3 =====
    case 'u': sendZClawCommand(CMD_Z_UP, FULL_SPEED); break;
    case 'j': sendZClawCommand(CMD_Z_DOWN, FULL_SPEED); break;
    case 'o': sendZClawCommand(CMD_CLAW_OPEN, 0); break;
    case 'p': sendZClawCommand(CMD_CLAW_CLOSE, 0); break;
    case 't': sendZClawCommand(CMD_PLATFORM_TILT, 0); break;
    case 'y': sendZClawCommand(CMD_PLATFORM_FLAT, 0); break;
    case 'h': sendZClawCommand(CMD_HOME, 0); break;

    // ===== 全部停止 =====
    case '0':
      sendLegCommand(CMD_LEG_STOP, 0);
      sendGripperCommand(CMD_GRIPPER_STOP, 0);
      sendZClawCommand(CMD_ZCLAW_STOP, 0);
      break;

    default:
      if (c != '\n' && c != '\r') {
        Serial.print("Unknown cmd: ");
        Serial.println(c);
      }
      break;
  }
}

// =====================================================
//  Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("========================================");
  Serial.println("  ESP32 MainController — Command Hub");
  Serial.println("  BT Recv + ESP-NOW Dispatch (3 peers)");
  Serial.println("========================================");

  // ----- Wi-Fi STA mode for ESP-NOW -----
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // ----- 藍芽初始化 -----
  SerialBT.begin("PickupRobot");  // 藍芽裝置名稱
  Serial.println("Bluetooth started: PickupRobot");

  // ----- ESP-NOW 初始化 -----
  if (esp_now_init() != ESP_OK) {
    Serial.println("!! ESP-NOW Init FAILED");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  // ----- 註冊 3 個子控板 Peer -----
  addPeer(Leg_Address, "C3#1 Leg");
  addPeer(r_theta_Address, "C3#2 R_Theta");
  addPeer(z_clap_Address, "C3#3 Z_Claw");

  Serial.println();
  Serial.println("ESP-NOW Initialized. Waiting for BT commands...");
  Serial.println("Use Serial Monitor or Bluetooth App to send commands.");
  Serial.println();
}

// =====================================================
//  Main Loop
// =====================================================
void loop() {
  // ----- 處理藍芽接收 -----
  if (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print("[BT] ");
    processCommand(c);
  }

  // ----- Serial 手動測試（debug 用）-----
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("[Serial] ");
    processCommand(c);
  }

  delay(10);
>>>>>>> e993a25612e77d4dc14e1e2119fa60f8b33a3675
}
