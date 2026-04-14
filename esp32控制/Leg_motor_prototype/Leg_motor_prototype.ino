#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

const int freq=30000;
const int pwmChannel=0;
const int resolution=8;//8 bit PWM=0~255
const int leftmotor=0;
const int rightmotor=1;
int dutyCycle=200;//speedvalur



// =========================
// 左馬達 (L298N Motor A)
// =========================
const int Left_Pin1=16;
const int Left_Pin2=17;
const int Left_enable=18;

// =========================
// 右馬達 (L298N Motor B)
// =========================
const int Right_Pin1=14;
const int Right_Pin2=26;
const int Right_enable=27;

// =========================
// 馬達控制函式
// =========================
//------停止------
void StopMotor(){
  digitalWrite(Left_Pin1,LOW);
  digitalWrite(Left_Pin2,LOW);

  digitalWrite(Right_Pin1,LOW);
  digitalWrite(Right_Pin2,LOW);

  ledcWrite(Left_enable,dutycycle);
  ledcWrite(Right_enable,dutycycle);
}

//------前進------
void Forward(){
  //左正轉
  digitalWrite(Left_Pin1,HIGH);
  digitalWrite(Left_Pin2,LOW);
  //右正轉
  digitalWrite(Right_Pin1,HIGH);
  digitalWrite(Right_Pin2,LOW);

  ledcWrite(Left_enable,dutycycle);
  ledcWrite(Right_enable,dutycycle);
}

//左轉: 左邊前進，右邊後退
void LeftTurn(){
  digitalWrite(Left_Pin1,HIGH);
  digitalWrite(Left_Pin2,LOW);

  digitalWrite(Right_Pin1,LOW);
  digitalWrite(Right_Pin2,HIGH);

  ledcWrite(Left_enable,dutycycle);
  ledWrite(Right_enable,dutycycle);
}

//右轉: 右邊前進，左邊後退
void RightTurn(){
  digitalWrite(Left_Pin1,LOW);
  digitalWrite(Left_Pin2,HIGH);

  digitalWrite(Right_Pin1,HIGH);
  digitalWrite(Right_Pin2,LOW);

  ledcWrite(Left_enable,dutycycle);
  ledWrite(Right_enable,dutycycle);
}


// =========================
// 設定對應的角位
// setup: 開機只執行一次
// =========================
void setup() {
  // 設定GPIO的 I/O模式
  pinMode(Left_Pin1,Output);
  pinMode(Left_Pin2,Output);
  pinMode(Left_enable,Output);

  pinMode(Right_Pin1,Output);
  pinMode(Right_Pin2,Outout);
  pinMode(Right_enable,Output)
  /*設定 ESP32的 LEDC PWM功能:
    enable1_Pin -> PWM輸出腳位
    freq -> PWM頻率
    resolution -> PWM解析度
    pwmChannel -> 通道*/
  ledcAttachChannel(enable1_Pin, freq,resolution,pwmChannel);//esp32 pwm control
  ledcAttachChannel(enable1_Pin, freq,resolution,pwmChannel);//esp32 pwm control

  StopMotors();

  //啟動序列埠 (用來debug)
  Serial.begin(115200);
  Serial.print("Testing DC Motor...")
}



// =========================
// loop: 重複執行
// =========================
void loop() {
  //按鈕 -> bool值來確定執行
  bool forwardPress=(digitalRead()==LOW);
  bool backwardPress=(digitalRead()==LOW);
  bool LeftPress=(digitalRead()==LOW);
  bool RightPress=(digitalRead()==LOW);
  
  //bool value=1 做動
}
