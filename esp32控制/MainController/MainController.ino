#include <esp_now.h>
#include <WiFi.h>

//接收端 MAC地址
uint8_t Leg_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t r_theta_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t z_clap_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

//-------腿部機構指令列舉-------
enum Leg_CommandType{
  CMD_STOP=0,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_LEFT,
  CMD_RIGHT};
//-------ESP-NOW 腿部傳輸資料格式-------
typedef struct Leg_now_message{
  uint8_t command;
  uint8_t speed;
}Leg_now_message;

Leg_now_message leg_msg;

//-------目前指令與速度-------
//資料送出後，系統自動呼叫函式，告訴這次傳輸成功或失敗
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  //wifi_tx_info_t: 傳送資訊的指標，放置傳送的MAC地址
  if(memcpy(info->mac, Leg_Address, 6) == 0){
    Serial.print("Last Packet Sent to: ");
    Serial.println("Leg");
  } else if(memcpy(info->mac, r_theta_Address, 6) == 0){
    Serial.print("Last Packet Sent to: ");
    Serial.println("R_Theta");
  } else if(memcpy(info->mac, z_clap_Address, 6) == 0){
    Serial.print("Last Packet Sent to: ");
    Serial.println("Z_Clap");
  } else {
    Serial.print("Last Packet Sent to: ");
    Serial.println("Unknown");
  }

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return ;
}
esp_rigister_send_cb(OnDataSent);

esp_now_peer_info_t peerInfo={};
memcpy(peerInfo.peer_addr, Leg_Address, 6);
peerInfo.channel=0;
peerInfo.encrypt=false;

if(esp_now_add_peer(&peerInfo)!=ESP_OK){
  Serial.println("Failed to add peer");
  return;
  }
Serial.println("ESP-NOW Initialized");
}


void loop() {
  // put your main code here, to run repeatedly:
  //測試傳送資料
  leg_msg.speed=200;

  leg_msg.command=CMD_FORWARD;
  esp_now_send(Leg_Address, (uint8_t *) &leg_msg, sizeof(leg_msg));
  delay(1000);

  leg_msg.command=CMD_BACKWARD;
  esp_now_send(Leg_Address, (uint8_t *) &leg_msg, sizeof(leg_msg));
  delay(1000);

  leg_msg.command=CMD_LEFT;
  esp_now_send(Leg_Address, (uint8_t *) &leg_msg, sizeof(leg_msg));
  delay(1000);

  leg_msg.command=CMD_RIGHT;
  esp_now_send(Leg_Address, (uint8_t *) &leg_msg, sizeof(leg_msg));
  delay(1000);
  
}
