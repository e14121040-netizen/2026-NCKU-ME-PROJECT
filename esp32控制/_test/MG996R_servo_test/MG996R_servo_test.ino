/*
 * =====================================================
 *  MG996R Servo 測試 — ESP32-C3 Super Mini
 * =====================================================
 *
 *  純 servo 測試程式，不含 ESP-NOW、L298N 等其他功能。
 *  透過 Serial Monitor 輸入指令控制 MG996R。
 *
 *  接線：
 *  ESP32-C3 Super Mini   ->   MG996R
 *  ─────────────────────────────────
 *  GPIO 4  ->  信號線（橘/黃色）
 *  不接     ->  V+（紅色）接外部 5V 電源
 *  GND     ->  GND（棕/黑色）共地
 *
 *  ⚠️ 重要：MG996R 堵轉電流可達 2.5A，
 *     絕對不可從 ESP32-C3 的 3.3V/5V 取電！
 *     必須使用外部 5V 穩壓電源（至少 2A）。
 *     ESP32-C3 的 GND 要跟外部電源 GND 共地。
 *
 *  Serial 快捷鍵（115200 baud）：
 *    'a' = 自動來回掃描 (0° -> 180° -> 0°)
 *    'o' = 轉到 0°
 *    'c' = 轉到 90°（中位）
 *    'f' = 轉到 180°
 *    '0'~'9' = 快速角度 (0°, 20°, 40°, ..., 180°)
 *    '+' = 角度 +10°
 *    '-' = 角度 -10°
 *    '?' = 顯示狀態
 */

#include <ESP32Servo.h>

// =====================================================
//  設定
// =====================================================
const int SERVO_PIN = 4;   // GPIO 4 接 MG996R 信號線

Servo myServo;

int currentAngle = 90;     // 初始角度（中位）
bool sweepMode = false;    // 自動掃描模式
int sweepAngle = 0;        // 掃描角度
int sweepDir = 1;          // 掃描方向 (+1/-1)
unsigned long lastSweep = 0;
const int SWEEP_DELAY = 15; // 掃描每步延遲 (ms)

// =====================================================
//  Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("========================================");
  Serial.println("  MG996R Servo Test - ESP32-C3");
  Serial.println("========================================");

  // ----- Servo 初始化 -----
  // MG996R 標準脈寬：500~2500 μs
  myServo.setPeriodHertz(50);           // 50Hz (標準 servo)
  myServo.attach(SERVO_PIN, 500, 2500); // min/max pulse width

  // 移到中位
  myServo.write(currentAngle);
  Serial.print("Servo attached on GPIO ");
  Serial.println(SERVO_PIN);
  Serial.print("Initial angle: ");
  Serial.print(currentAngle);
  Serial.println("°");

  // ----- 使用說明 -----
  Serial.println();
  Serial.println("--- Commands ---");
  Serial.println("  a = Auto sweep (0-180-0)");
  Serial.println("  o = Go to 0 deg");
  Serial.println("  c = Go to 90 deg (center)");
  Serial.println("  f = Go to 180 deg");
  Serial.println("  0~9 = Quick angle (0,20,40,...,180)");
  Serial.println("  + = Angle +10 deg");
  Serial.println("  - = Angle -10 deg");
  Serial.println("  ? = Show status");
  Serial.println("----------------");
  Serial.println();
}

// =====================================================
//  移動到指定角度
// =====================================================
void goToAngle(int angle) {
  currentAngle = constrain(angle, 0, 180);
  myServo.write(currentAngle);
  Serial.print(">> Angle: ");
  Serial.print(currentAngle);
  Serial.println("°");
}

// =====================================================
//  自動掃描更新
// =====================================================
void updateSweep() {
  if (!sweepMode) return;
  if (millis() - lastSweep < SWEEP_DELAY) return;

  lastSweep = millis();
  sweepAngle += sweepDir;

  if (sweepAngle >= 180) {
    sweepAngle = 180;
    sweepDir = -1;
  } else if (sweepAngle <= 0) {
    sweepAngle = 0;
    sweepDir = 1;
  }

  myServo.write(sweepAngle);
  currentAngle = sweepAngle;

  // 每 30° 印一次
  if (sweepAngle % 30 == 0) {
    Serial.print("Sweep: ");
    Serial.print(sweepAngle);
    Serial.println("°");
  }
}

// =====================================================
//  Main Loop
// =====================================================
void loop() {
  // 自動掃描
  updateSweep();

  // Serial 指令
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'a':
        sweepMode = !sweepMode;
        if (sweepMode) {
          sweepAngle = currentAngle;
          sweepDir = 1;
          Serial.println(">> Sweep ON");
        } else {
          Serial.println(">> Sweep OFF");
          Serial.print("   Stopped at ");
          Serial.print(currentAngle);
          Serial.println("°");
        }
        break;

      case 'o':
        sweepMode = false;
        goToAngle(0);
        break;

      case 'c':
        sweepMode = false;
        goToAngle(90);
        break;

      case 'f':
        sweepMode = false;
        goToAngle(180);
        break;

      case '+':
        sweepMode = false;
        goToAngle(currentAngle + 10);
        break;

      case '-':
        sweepMode = false;
        goToAngle(currentAngle - 10);
        break;

      // 數字鍵快速角度：0=0°, 1=20°, ..., 9=180°
      case '0': sweepMode = false; goToAngle(0);   break;
      case '1': sweepMode = false; goToAngle(20);  break;
      case '2': sweepMode = false; goToAngle(40);  break;
      case '3': sweepMode = false; goToAngle(60);  break;
      case '4': sweepMode = false; goToAngle(80);  break;
      case '5': sweepMode = false; goToAngle(100); break;
      case '6': sweepMode = false; goToAngle(120); break;
      case '7': sweepMode = false; goToAngle(140); break;
      case '8': sweepMode = false; goToAngle(160); break;
      case '9': sweepMode = false; goToAngle(180); break;

      case '?':
        Serial.println("--- Status ---");
        Serial.print("  Angle: ");
        Serial.print(currentAngle);
        Serial.println("°");
        Serial.print("  Sweep: ");
        Serial.println(sweepMode ? "ON" : "OFF");
        Serial.print("  GPIO: ");
        Serial.println(SERVO_PIN);
        Serial.println("--------------");
        break;

      default:
        if (c != '\n' && c != '\r') {
          Serial.print("Unknown key: ");
          Serial.println(c);
        }
        break;
    }
  }

  delay(5);
}
