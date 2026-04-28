#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const uint8_t ESPNOW_CHANNEL = 1;

const int freq=2000;
const int resolution=8;//8 bit PWM=0~255
const int leftMotorChannel=0;
const int rightMotorChannel=1;
int dutyCycle=200;//speedvalue
const int minRunningPwm=90;
const int startBoostPwm=140;
const unsigned long startBoostMs=120;



// =========================
// 左馬達 (L298N Motor A)
// =========================
// ESP32-C3 常用可輸出 GPIO，避開不存在或容易衝突的腳位
const int Left_Pin1=0;
const int Left_Pin2=1;
const int Left_enable=2;

// =========================
// 右馬達 (L298N Motor B)
// =========================
const int Right_Pin1=3;
const int Right_Pin2=4;
const int Right_enable=5;

// =========================
// 指令列舉
// =========================
enum CommandType{
  CMD_STOP=0,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_LEFT,
  CMD_RIGHT,
  };

//ESP-NOW 傳輸資料格式
typedef struct now_message{
  uint8_t command;
  uint8_t speed;
}now_message;

typedef struct ack_message{
  uint8_t command;
  uint8_t speed;
  uint8_t status;
}ack_message;

volatile uint8_t currentCommand=CMD_STOP;//預設停止
volatile uint8_t currentSpeed=200;//pwm值，0~255，預設200 占空比80%
bool espNowReady=false;
uint8_t lastSenderMac[6]={0};
bool senderKnown=false;
int lastLeftSpeed=0;
int lastRightSpeed=0;

void printMacAddress(const uint8_t *mac){
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

//======確保對方已經註冊為 ESP-NOW peer，如果沒有就註冊======
bool ensurePeerExists(const uint8_t *peerMac){
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

void sendAck(){
  if (!senderKnown) {
    return;
  }

  if (!ensurePeerExists(lastSenderMac)) {
    return;
  }

  ack_message ack = {
    currentCommand,
    currentSpeed,
    1
  };

  esp_err_t result = esp_now_send(lastSenderMac, (uint8_t *)&ack, sizeof(ack));
  if (result != ESP_OK) {
    Serial.print("ACK send failed, err=");
    Serial.println(result);
  }
}


// =========================
// 馬達控制函式
// =========================

//------馬達速度控制------
void SetMotorSpeed(int left_speed, int right_speed){
  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  if (left_speed > 0 && left_speed < minRunningPwm) {
    left_speed = minRunningPwm;
  }
  if (right_speed > 0 && right_speed < minRunningPwm) {
    right_speed = minRunningPwm;
  }

  // Give the motor a short kick when starting from standstill.
  if (left_speed > 0 && lastLeftSpeed == 0) {
    ledcWrite(Left_enable, max(left_speed, startBoostPwm));
    delay(startBoostMs);
  }
  if (right_speed > 0 && lastRightSpeed == 0) {
    ledcWrite(Right_enable, max(right_speed, startBoostPwm));
    delay(startBoostMs);
  }

  ledcWrite(Left_enable, left_speed);
  ledcWrite(Right_enable, right_speed);
  lastLeftSpeed = left_speed;
  lastRightSpeed = right_speed;
}

//------停止------
void StopMotor(){
  digitalWrite(Left_Pin1,LOW);
  digitalWrite(Left_Pin2,LOW);

  digitalWrite(Right_Pin1,LOW);
  digitalWrite(Right_Pin2,LOW);

  SetMotorSpeed(0, 0);
}

//------前進------
void Forward(){
  //左正轉
  digitalWrite(Left_Pin1,HIGH);
  digitalWrite(Left_Pin2,LOW);
  //右正轉
  digitalWrite(Right_Pin1,HIGH);
  digitalWrite(Right_Pin2,LOW);

  SetMotorSpeed(dutyCycle, dutyCycle);
}

//------後退------
void Backward(){
  digitalWrite(Left_Pin1,LOW);
  digitalWrite(Left_Pin2,HIGH);

  digitalWrite(Right_Pin1,LOW);
  digitalWrite(Right_Pin2,HIGH);

  SetMotorSpeed(dutyCycle, dutyCycle);
}

//左轉: 左邊前進，右邊後退
void LeftTurn(){
  digitalWrite(Left_Pin1,HIGH);
  digitalWrite(Left_Pin2,LOW);

  digitalWrite(Right_Pin1,LOW);
  digitalWrite(Right_Pin2,HIGH);

  SetMotorSpeed(dutyCycle, dutyCycle);
}

//右轉: 右邊前進，左邊後退
void RightTurn(){
  digitalWrite(Left_Pin1,LOW);
  digitalWrite(Left_Pin2,HIGH);

  digitalWrite(Right_Pin1,HIGH);
  digitalWrite(Right_Pin2,LOW);

  SetMotorSpeed(dutyCycle, dutyCycle);
}


// =====================================================
// ESP-NOW 接收 callback
// 注意：這裡不要做太耗時的事情
// =====================================================
void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len){
  if (incomingData == nullptr || len != sizeof(now_message)) {
    Serial.print("Invalid ESP-NOW packet, len=");
    Serial.println(len);
    return;
  }

  now_message incomingMessage;
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));

  if (recvInfo != nullptr && recvInfo->src_addr != nullptr) {
    memcpy(lastSenderMac, recvInfo->src_addr, 6);
    senderKnown = true;
  }

  currentCommand=incomingMessage.command;
  currentSpeed=incomingMessage.speed;

  Serial.print("Received command=");
  Serial.print(currentCommand);
  Serial.print(", speed=");
  Serial.print(currentSpeed);
  Serial.print(", from=");
  printMacAddress(lastSenderMac);
  Serial.println();

  sendAck();
}

void setupEspNow(){
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

  esp_now_register_recv_cb(OnDataRecv);
  espNowReady=true;
  Serial.println("ESP-NOW receiver ready");
}

// =========================
// 設定對應的角位
// setup: 開機只執行一次
// =========================
void setup() {
  //啟動序列埠 (用來debug)
  Serial.begin(115200);
  delay(300);

  // 設定GPIO的 I/O模式
  pinMode(Left_Pin1,OUTPUT);
  pinMode(Left_Pin2,OUTPUT);
  pinMode(Left_enable,OUTPUT);

  pinMode(Right_Pin1,OUTPUT);
  pinMode(Right_Pin2,OUTPUT);
  pinMode(Right_enable,OUTPUT);
  /*設定 ESP32的 LEDC PWM功能:
    Left_enable / Right_enable -> PWM輸出腳位
    freq -> PWM頻率
    resolution -> PWM解析度
    leftMotorChannel / rightMotorChannel -> PWM通道*/
  ledcAttachChannel(Left_enable, freq, resolution, leftMotorChannel);//esp32 pwm control
  ledcAttachChannel(Right_enable, freq, resolution, rightMotorChannel);//esp32 pwm control

  StopMotor();
  Serial.println("Testing DC Motor...");
  setupEspNow();
  Serial.print("Receiver MAC (STA): ");
  Serial.println(WiFi.macAddress());
}



// =========================
// loop: 重複執行
// =========================
void loop() {
  dutyCycle=currentSpeed;

  //按鈕 -> bool值來確定執行
  switch (currentCommand){
    case CMD_STOP:
      StopMotor();
      break;
    case CMD_FORWARD:
      Forward();
      break;
    case CMD_BACKWARD:
      Backward();
      break;
    case CMD_LEFT:
      LeftTurn();
      break;
    case CMD_RIGHT:
      RightTurn();  
      break;
    default:
      StopMotor();
      break;
  }
  
  delay(20);
}
