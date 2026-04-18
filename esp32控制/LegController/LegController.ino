/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人 腿部子控板
 *  ESP32-C3 #1 LegController
 * =====================================================
 *
 *  功能：
 *  - 透過 ESP-NOW 接收主控板（ESP32）腿部指令
 *  - 控制左右兩顆 JGB37-520 直流減速馬達（差速轉向）
 *  - 使用 L298N 驅動馬達
 *  - Serial Monitor 手動測試模式
 *
 *  指令協議（ESP-NOW 接收 / Serial 手動測試）：
 *    CMD_LEG_STOP    = 0  → 停止
 *    CMD_FORWARD     = 1  → 前進（左右正轉）
 *    CMD_BACKWARD    = 2  → 後退（左右反轉）
 *    CMD_LEFT        = 3  → 左轉（左慢右快）
 *    CMD_RIGHT       = 4  → 右轉（左快右慢）
 *    CMD_SPIN_LEFT   = 5  → 左旋（左反右正）
 *    CMD_SPIN_RIGHT  = 6  → 右旋（左正右反）
 *
 *  Serial 快捷鍵：
 *    'f'=前進 'b'=後退 'l'=左轉 'r'=右轉
 *    'q'=左旋 'e'=右旋 '0'=停止
 *    '+'=加速 '-'=減速 '?'=狀態
 *
 *  硬體接線（ESP32-C3 Super Mini → L298N）：
 *  GPIO 0  → IN1 (左馬達)
 *  GPIO 1  → IN2
 *  GPIO 3  → ENA (PWM)
 *  GPIO 4  → IN3 (右馬達)
 *  GPIO 5  → IN4
 *  GPIO 6  → ENB (PWM)
 *  GND     → GND (共地)
 */

#include <esp_now.h>
#include <WiFi.h>

// =====================================================
//  ESP-NOW 資料格式（與主控板 MainController 一致）
// =====================================================
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

leg_now_message incomingMsg;
bool newDataReceived = false;

// =====================================================
//  L298N 腳位定義 — ESP32-C3 Super Mini
// =====================================================
// 左馬達 (Motor A)
const int motorL_Pin1 = 0;
const int motorL_Pin2 = 1;
const int motorL_EN   = 3;   // PWM

// 右馬達 (Motor B)
const int motorR_Pin1 = 4;
const int motorR_Pin2 = 5;
const int motorR_EN   = 6;   // PWM

// =====================================================
//  PWM 設定
// =====================================================
const int pwmFreq       = 30000;
const int pwmResolution = 8;      // 8-bit → 0~255
const int pwmChannel_L  = 0;
const int pwmChannel_R  = 1;

// =====================================================
//  速度設定
// =====================================================
int dutyCycle = 200;              // 預設速度
const int SPEED_MIN  = 80;
const int SPEED_MAX  = 255;
const int SPEED_STEP = 20;
const int TURN_RATIO = 60;       // 轉向時慢側速度百分比

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
//  馬達控制函式
// =====================================================

void stopMotors() {
  digitalWrite(motorL_Pin1, LOW);
  digitalWrite(motorL_Pin2, LOW);
  digitalWrite(motorR_Pin1, LOW);
  digitalWrite(motorR_Pin2, LOW);
  ledcWrite(motorL_EN, 0);
  ledcWrite(motorR_EN, 0);
  Serial.println("=== STOP ===");
}

void forward(int spd) {
  digitalWrite(motorL_Pin1, HIGH);
  digitalWrite(motorL_Pin2, LOW);
  digitalWrite(motorR_Pin1, HIGH);
  digitalWrite(motorR_Pin2, LOW);
  ledcWrite(motorL_EN, spd);
  ledcWrite(motorR_EN, spd);
  Serial.print("Forward, speed="); Serial.println(spd);
}

void backward(int spd) {
  digitalWrite(motorL_Pin1, LOW);
  digitalWrite(motorL_Pin2, HIGH);
  digitalWrite(motorR_Pin1, LOW);
  digitalWrite(motorR_Pin2, HIGH);
  ledcWrite(motorL_EN, spd);
  ledcWrite(motorR_EN, spd);
  Serial.print("Backward, speed="); Serial.println(spd);
}

void turnLeft(int spd) {
  int slowSpeed = spd * TURN_RATIO / 100;
  digitalWrite(motorL_Pin1, HIGH);
  digitalWrite(motorL_Pin2, LOW);
  digitalWrite(motorR_Pin1, HIGH);
  digitalWrite(motorR_Pin2, LOW);
  ledcWrite(motorL_EN, slowSpeed);  // 左慢
  ledcWrite(motorR_EN, spd);        // 右快
  Serial.print("Left Turn, L="); Serial.print(slowSpeed);
  Serial.print(", R="); Serial.println(spd);
}

void turnRight(int spd) {
  int slowSpeed = spd * TURN_RATIO / 100;
  digitalWrite(motorL_Pin1, HIGH);
  digitalWrite(motorL_Pin2, LOW);
  digitalWrite(motorR_Pin1, HIGH);
  digitalWrite(motorR_Pin2, LOW);
  ledcWrite(motorL_EN, spd);        // 左快
  ledcWrite(motorR_EN, slowSpeed);  // 右慢
  Serial.print("Right Turn, L="); Serial.print(spd);
  Serial.print(", R="); Serial.println(slowSpeed);
}

void spinLeft(int spd) {
  // 左反轉、右正轉 → 原地左旋
  digitalWrite(motorL_Pin1, LOW);
  digitalWrite(motorL_Pin2, HIGH);
  digitalWrite(motorR_Pin1, HIGH);
  digitalWrite(motorR_Pin2, LOW);
  ledcWrite(motorL_EN, spd);
  ledcWrite(motorR_EN, spd);
  Serial.print("Spin Left, speed="); Serial.println(spd);
}

void spinRight(int spd) {
  // 左正轉、右反轉 → 原地右旋
  digitalWrite(motorL_Pin1, HIGH);
  digitalWrite(motorL_Pin2, LOW);
  digitalWrite(motorR_Pin1, LOW);
  digitalWrite(motorR_Pin2, HIGH);
  ledcWrite(motorL_EN, spd);
  ledcWrite(motorR_EN, spd);
  Serial.print("Spin Right, speed="); Serial.println(spd);
}

// =====================================================
//  處理指令
// =====================================================
void executeCommand(uint8_t cmd, uint8_t spd) {
  if (spd > 0) {
    dutyCycle = constrain(spd, SPEED_MIN, SPEED_MAX);
  }

  switch (cmd) {
    case CMD_LEG_STOP:   stopMotors(); break;
    case CMD_FORWARD:    forward(dutyCycle); break;
    case CMD_BACKWARD:   backward(dutyCycle); break;
    case CMD_LEFT:       turnLeft(dutyCycle); break;
    case CMD_RIGHT:      turnRight(dutyCycle); break;
    case CMD_SPIN_LEFT:  spinLeft(dutyCycle); break;
    case CMD_SPIN_RIGHT: spinRight(dutyCycle); break;
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

  Serial.println("========================================");
  Serial.println("  ESP32-C3 #1 Leg Controller");
  Serial.println("  Walking Motor Driver (L298N)");
  Serial.println("========================================");

  // ----- 馬達腳位設定 -----
  pinMode(motorL_Pin1, OUTPUT);
  pinMode(motorL_Pin2, OUTPUT);
  pinMode(motorL_EN,   OUTPUT);
  pinMode(motorR_Pin1, OUTPUT);
  pinMode(motorR_Pin2, OUTPUT);
  pinMode(motorR_EN,   OUTPUT);

  // ----- PWM 設定 -----
  ledcAttachChannel(motorL_EN, pwmFreq, pwmResolution, pwmChannel_L);
  ledcAttachChannel(motorR_EN, pwmFreq, pwmResolution, pwmChannel_R);

  // ----- 初始停止 -----
  stopMotors();

  // ----- ESP-NOW 初始化 -----
  WiFi.mode(WIFI_STA);
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
  Serial.println("  l=Left     r=Right");
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
    executeCommand(incomingMsg.command, incomingMsg.speed);
  }

  // ----- Serial 手動測試 -----
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'f': executeCommand(CMD_FORWARD, 0); break;
      case 'b': executeCommand(CMD_BACKWARD, 0); break;
      case 'l': executeCommand(CMD_LEFT, 0); break;
      case 'r': executeCommand(CMD_RIGHT, 0); break;
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
