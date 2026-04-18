#include <esp_now.h>
#include <WiFi.h>
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
  } else {
    Serial.print("Last Packet Sent to: ");
    Serial.println("Unknown");
  }

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

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
}
