// ===== ESP32 基本功能測試 =====

// 你可以改這個腳位（內建LED通常是2）
const int ledPin = 2;

// PWM設定
const int pwmChannel = 0;
const int pwmFreq = 5000;
const int pwmResolution = 8;

void setup() {
  Serial.begin(115200);
  delay(1000);  // 等Serial穩定

  Serial.println("ESP32 Test Start");

  // GPIO測試
  pinMode(ledPin, OUTPUT);

  // PWM設定
ledcAttach(ledPin, pwmFreq, pwmResolution);
ledcWrite(ledPin, pwmResolution);

  Serial.println("Setup Done");
}

void loop() {
  Serial.println("GPIO Blink Test");

  // ===== GPIO測試 =====
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);

  Serial.println("PWM Fade Test");

  // ===== PWM測試 =====
  for (int duty = 0; duty <= 255; duty++) {
    ledcWrite(pwmChannel, duty);
    delay(5);
  }

  for (int duty = 255; duty >= 0; duty--) {
    ledcWrite(pwmChannel, duty);
    delay(5);
  }

  Serial.println("Loop Done\n");
}