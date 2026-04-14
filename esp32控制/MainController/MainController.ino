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
  } else {
    Serial.print("Unknown");
  }
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " OK" : " FAIL");
}

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
}
