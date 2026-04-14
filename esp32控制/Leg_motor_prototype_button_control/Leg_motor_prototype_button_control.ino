// ============================================
// ESP32 + L298N + 4按鈕控制雙馬達小車
// 先把馬達控制做好，之後再接藍牙/BLE
// ============================================

// 目前先不啟用藍牙
// 注意：如果你之後用的是 ESP32-C3，BluetoothSerial 不能用
// #include <BluetoothSerial.h>
// BluetoothSerial SerialBT;

// =========================
// PWM 設定
// =========================
const int freq = 30000;
const int resolution = 8;      // 8-bit PWM -> 0 ~ 255
const int leftmotor_channel = 0;
const int rightmotor_channel = 1;
int dutyCycle = 200;           // 速度大小

// =========================
// 左馬達 (L298N Motor A)
// =========================
const int motor1_Pin1 = 16;
const int motor1_Pin2 = 17;
const int enable1_Pin = 18;

// =========================
// 右馬達 (L298N Motor B)
// =========================
const int motor2_Pin1 = 14;
const int motor2_Pin2 = 26;
const int enable2_Pin = 27;

// =========================
// 按鈕腳位
// 接法：GPIO ---- 按鈕 ---- GND
// 使用 INPUT_PULLUP：
// 沒按 = HIGH，按下 = LOW
// =========================
const int btnForward  = 32;
const int btnBackward = 33;
const int btnLeft     = 25;
const int btnRight    = 13;

// =========================
// 馬達控制函式
// =========================
void stopMotors() {
  digitalWrite(motor1_Pin1, LOW);
  digitalWrite(motor1_Pin2, LOW);
  digitalWrite(motor2_Pin1, LOW);
  digitalWrite(motor2_Pin2, LOW);

  ledcWrite(enable1_Pin, 0);
  ledcWrite(enable2_Pin, 0);
}

void Forward() {
  // 左馬達正轉
  digitalWrite(motor1_Pin1, HIGH);
  digitalWrite(motor1_Pin2, LOW);

  // 右馬達正轉
  digitalWrite(motor2_Pin1, HIGH);
  digitalWrite(motor2_Pin2, LOW);

  ledcWrite(enable1_Pin, dutyCycle);
  ledcWrite(enable2_Pin, dutyCycle);
}

void Backward() {
  // 左馬達反轉
  digitalWrite(motor1_Pin1, LOW);
  digitalWrite(motor1_Pin2, HIGH);

  // 右馬達反轉
  digitalWrite(motor2_Pin1, LOW);
  digitalWrite(motor2_Pin2, HIGH);

  ledcWrite(enable1_Pin, dutyCycle);
  ledcWrite(enable2_Pin, dutyCycle);
}

void LeftTurn() {
  // 原地左轉：左輪後退，右輪前進
  digitalWrite(motor1_Pin1, LOW);
  digitalWrite(motor1_Pin2, HIGH);

  digitalWrite(motor2_Pin1, HIGH);
  digitalWrite(motor2_Pin2, LOW);

  ledcWrite(enable1_Pin, dutyCycle);
  ledcWrite(enable2_Pin, dutyCycle);
}

void RightTurn() {
  // 原地右轉：左輪前進，右輪後退
  digitalWrite(motor1_Pin1, HIGH);
  digitalWrite(motor1_Pin2, LOW);

  digitalWrite(motor2_Pin1, LOW);
  digitalWrite(motor2_Pin2, HIGH);

  ledcWrite(enable1_Pin, dutyCycle);
  ledcWrite(enable2_Pin, dutyCycle);
}

// =========================
// setup: 開機只執行一次
// =========================
void setup() {
  Serial.begin(115200);

  // 馬達腳位設定
  pinMode(motor1_Pin1, OUTPUT);
  pinMode(motor1_Pin2, OUTPUT);
  pinMode(enable1_Pin, OUTPUT);

  pinMode(motor2_Pin1, OUTPUT);
  pinMode(motor2_Pin2, OUTPUT);
  pinMode(enable2_Pin, OUTPUT);

  // 按鈕腳位設定
  pinMode(btnForward, INPUT_PULLUP);
  pinMode(btnBackward, INPUT_PULLUP);
  pinMode(btnLeft, INPUT_PULLUP);
  pinMode(btnRight, INPUT_PULLUP);

  // 設定 ESP32 PWM
  ledcAttachChannel(enable1_Pin, freq, resolution, leftmotor_channel);
  ledcAttachChannel(enable2_Pin, freq, resolution, rightmotor_channel);

  stopMotors();

  Serial.println("Button motor control ready");
  Serial.println("Press button: Forward / Backward / Left / Right");
}

// =========================
// loop: 重複執行
// =========================
void loop() {
  bool forwardPressed  = (digitalRead(btnForward)  == LOW);
  bool backwardPressed = (digitalRead(btnBackward) == LOW);
  bool leftPressed     = (digitalRead(btnLeft)     == LOW);
  bool rightPressed    = (digitalRead(btnRight)    == LOW);

  if (forwardPressed) {
    Serial.println("Moving Forward");
    Forward();
  }
  else if (backwardPressed) {
    Serial.println("Moving Backward");
    Backward();
  }
  else if (leftPressed) {
    Serial.println("Turning Left");
    LeftTurn();
  }
  else if (rightPressed) {
    Serial.println("Turning Right");
    RightTurn();
  }
  else {
    stopMotors();
  }

  delay(20); // 簡單防彈跳 + 降低 Serial 狂刷
}
