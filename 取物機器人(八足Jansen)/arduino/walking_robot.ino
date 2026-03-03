/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — 取物機器人（八足 Theo Jansen）
 *  主控板：Arduino MEGA 2560
 * =====================================================
 *
 *  系統架構：
 *  手機 App (App Inventor) → HC-05 藍芽 → MEGA
 *    → L298N  → 左右直流馬達（八足步行）
 *    → PCA9685 → 伺服馬達群（機械手臂 + 夾爪）
 *
 *  步行機構：Theo Jansen Linkage 八足（Spider Robot）
 *  - 左側 4 足 → 馬達 A
 *  - 右側 4 足 → 馬達 B
 *  - 交替步態（alternating gait）— 同側每足相位差 90°
 *  - 差速轉向（與輪式底盤邏輯相同）
 *
 *  指令協議（藍芽接收單字元）：
 *  步行控制：
 *    'f' = 前進      'b' = 後退
 *    'l' = 左轉      'r' = 右轉
 *    'F' = 前進(慢)   'B' = 後退(慢)
 *    'L' = 左轉(慢)   'R' = 右轉(慢)
 *    'q' = 左旋轉     'e' = 右旋轉
 *    '0' = 停止
 *  手臂控制：
 *    'u' = 肩上升     'd' = 肩下降
 *    'i' = 肘上升     'k' = 肘下降
 *    'o' = 爪張開     'p' = 爪閉合
 *    'h' = 手臂歸位
 *    'v' = 回傳電池電壓
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

// =====================================================
//  腳位定義 — L298N 步行馬達
// =====================================================
// 馬達 A：左側 4 足
#define MOTOR_L_IN1 40
#define MOTOR_L_IN2 41
#define MOTOR_L_EN 9

// 馬達 B：右側 4 足
#define MOTOR_R_IN1 42
#define MOTOR_R_IN2 43
#define MOTOR_R_EN 10

// =====================================================
//  電池電壓監測
// =====================================================
// 用分壓器 (例如 30kΩ + 10kΩ) 將電池電壓降到 Arduino 可讀範圍
// Vbat = analogRead * (5.0/1023) * ((R1+R2)/R2)
#define BATTERY_PIN A0
#define VOLTAGE_DIVIDER 4.0  // (30k+10k)/10k = 4.0，依實際電阻調整
#define LOW_VOLTAGE 10.0     // 3S LiPo 低電壓警戒 (3.33V/cell)
#define CRITICAL_VOLTAGE 9.6 // 強制停機電壓 (3.2V/cell)

// LED 狀態指示
#define LED_PIN 13 // 內建 LED

// =====================================================
//  PCA9685 伺服馬達通道定義
// =====================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // 預設 I2C 位址 0x40

#define SERVO_FREQ 50
#define SERVO_MIN 125 // 0° 脈衝寬度（out of 4096）
#define SERVO_MAX 575 // 180° 脈衝寬度

// 伺服馬達通道分配
#define CH_SHOULDER 0 // 肩關節（升降）
#define CH_ELBOW 1    // 肘關節（伸縮）
#define CH_CLAW 2     // 夾爪（開合）
// 預留 CH 3~5 給額外伺服

// =====================================================
//  手臂角度設定（依實際組裝調整）
// =====================================================
// 肩關節
int shoulderAngle = 90;
const int SHOULDER_MIN = 30;
const int SHOULDER_MAX = 150;
const int SHOULDER_HOME = 90;
const int SHOULDER_STEP = 5;

// 肘關節
int elbowAngle = 90;
const int ELBOW_MIN = 30;
const int ELBOW_MAX = 150;
const int ELBOW_HOME = 90;
const int ELBOW_STEP = 5;

// 夾爪
int clawAngle = 90;
const int CLAW_OPEN = 40;   // 張開角度
const int CLAW_CLOSE = 110; // 閉合角度
const int CLAW_HOME = 40;

// =====================================================
//  速度設定
// =====================================================
const int SPEED_FULL = 200; // 全速 PWM 值 (0~255)
const int SPEED_HALF = 120; // 半速 PWM 值
const int SPEED_TURN = 150; // 轉向速度
const int RAMP_STEP = 10;   // 加減速每次增量
const int RAMP_DELAY = 15;  // 加減速間隔 (ms)

// 當前馬達速度追蹤（用於平滑加減速）
int currentSpeedL = 0;
int currentSpeedR = 0;

// 電壓監測計時
unsigned long lastVoltageCheck = 0;
const unsigned long VOLTAGE_CHECK_INTERVAL = 5000; // 每 5 秒檢測一次
bool lowVoltageWarning = false;

// =====================================================
//  馬達控制類別（複用學長的 Motor 類別改良版）
// =====================================================
class Motor {
private:
  int pinIN1, pinIN2, pinEN;
  int _currentSpeed; // 追蹤當前速度
public:
  Motor(int in1, int in2, int en) {
    pinIN1 = in1;
    pinIN2 = in2;
    pinEN = en;
    _currentSpeed = 0;
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);
    pinMode(pinEN, OUTPUT);
  }

  // speed: 正值=正轉, 負值=反轉, 0=停
  void setSpeed(int speed) {
    if (speed > 0) {
      digitalWrite(pinIN1, HIGH);
      digitalWrite(pinIN2, LOW);
    } else if (speed < 0) {
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, HIGH);
    } else {
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
    }
    _currentSpeed = speed;
    int pwm = constrain(abs(speed), 0, 255);
    analogWrite(pinEN, pwm);
  }

  // 平滑加減速到目標速度（避免突然啟停衝擊機構）
  void rampTo(int targetSpeed) {
    while (_currentSpeed != targetSpeed) {
      if (_currentSpeed < targetSpeed) {
        _currentSpeed = min(_currentSpeed + RAMP_STEP, targetSpeed);
      } else {
        _currentSpeed = max(_currentSpeed - RAMP_STEP, targetSpeed);
      }
      setSpeed(_currentSpeed);
      delay(RAMP_DELAY);
    }
  }

  int getCurrentSpeed() { return _currentSpeed; }

  void stop() { rampTo(0); }
};

// 建立馬達物件
Motor motorLeft(MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_EN);
Motor motorRight(MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_EN);

// =====================================================
//  工具函式
// =====================================================
int angleToPulse(int angle) { return map(angle, 0, 180, SERVO_MIN, SERVO_MAX); }

void setServo(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  pwm.setPWM(channel, 0, angleToPulse(angle));
}

// =====================================================
//  電池電壓監測
// =====================================================
float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  return raw * (5.0 / 1023.0) * VOLTAGE_DIVIDER;
}

void checkBattery() {
  float voltage = readBatteryVoltage();

  if (voltage < CRITICAL_VOLTAGE) {
    // 強制停機：電壓過低會損壞鋰電池
    stopWalking();
    armHome();
    Serial.println(F("!!! CRITICAL: Battery too low! Shutting down."));
    // 快速閃爍 LED 警告
    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  } else if (voltage < LOW_VOLTAGE) {
    if (!lowVoltageWarning) {
      lowVoltageWarning = true;
      Serial.print(F("WARNING: Low battery = "));
      Serial.print(voltage, 1);
      Serial.println(F("V"));
    }
    // 慢速閃爍提醒
    digitalWrite(LED_PIN, (millis() / 500) % 2);
  } else {
    lowVoltageWarning = false;
    digitalWrite(LED_PIN, HIGH); // 正常：LED 常亮
  }
}

// =====================================================
//  步行控制函式
// =====================================================
void walkForward(int speed) {
  motorLeft.rampTo(speed);
  motorRight.rampTo(speed);
}

void walkBackward(int speed) {
  motorLeft.rampTo(-speed);
  motorRight.rampTo(-speed);
}

void turnLeft(int speed) {
  motorLeft.setSpeed(speed / 3); // 左側慢
  motorRight.setSpeed(speed);    // 右側快
}

void turnRight(int speed) {
  motorLeft.setSpeed(speed);      // 左側快
  motorRight.setSpeed(speed / 3); // 右側慢
}

void spinLeft(int speed) {
  motorLeft.setSpeed(-speed); // 左側反轉
  motorRight.setSpeed(speed); // 右側正轉
}

void spinRight(int speed) {
  motorLeft.setSpeed(speed);   // 左側正轉
  motorRight.setSpeed(-speed); // 右側反轉
}

void stopWalking() {
  motorLeft.stop();
  motorRight.stop();
}

// =====================================================
//  手臂控制函式
// =====================================================
void armHome() {
  shoulderAngle = SHOULDER_HOME;
  elbowAngle = ELBOW_HOME;
  clawAngle = CLAW_HOME;
  setServo(CH_SHOULDER, shoulderAngle);
  setServo(CH_ELBOW, elbowAngle);
  setServo(CH_CLAW, clawAngle);
}

void shoulderUp() {
  shoulderAngle =
      constrain(shoulderAngle + SHOULDER_STEP, SHOULDER_MIN, SHOULDER_MAX);
  setServo(CH_SHOULDER, shoulderAngle);
}

void shoulderDown() {
  shoulderAngle =
      constrain(shoulderAngle - SHOULDER_STEP, SHOULDER_MIN, SHOULDER_MAX);
  setServo(CH_SHOULDER, shoulderAngle);
}

void elbowUp() {
  elbowAngle = constrain(elbowAngle + ELBOW_STEP, ELBOW_MIN, ELBOW_MAX);
  setServo(CH_ELBOW, elbowAngle);
}

void elbowDown() {
  elbowAngle = constrain(elbowAngle - ELBOW_STEP, ELBOW_MIN, ELBOW_MAX);
  setServo(CH_ELBOW, elbowAngle);
}

void clawOpen() {
  clawAngle = CLAW_OPEN;
  setServo(CH_CLAW, clawAngle);
}

void clawClose() {
  clawAngle = CLAW_CLOSE;
  setServo(CH_CLAW, clawAngle);
}

// =====================================================
//  Setup
// =====================================================
void setup() {
  Serial.begin(9600);   // Debug 用
  Serial1.begin(38400); // HC-05 藍芽（MEGA 的 Serial1 = Pin 18/19）

  // LED 狀態指示
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // 電池電壓初始檢測
  pinMode(BATTERY_PIN, INPUT);
  float initVoltage = readBatteryVoltage();
  Serial.print(F("Battery: "));
  Serial.print(initVoltage, 1);
  Serial.println(F("V"));

  // 初始化 PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // 手臂歸位
  armHome();

  // 確認開機
  Serial.println(F("=== Walking Robot Ready ==="));
  Serial.println(F("Waiting for Bluetooth commands..."));
}

// =====================================================
//  Main Loop — 接收藍芽指令並執行
// =====================================================
void loop() {
  // 定期檢測電池電壓
  if (millis() - lastVoltageCheck > VOLTAGE_CHECK_INTERVAL) {
    lastVoltageCheck = millis();
    checkBattery();
  }

  if (Serial1.available()) {
    char cmd = Serial1.read();
    Serial.print(F("CMD: "));
    Serial.println(cmd);

    switch (cmd) {
    // --------- 步行控制 ---------
    case 'f':
      walkForward(SPEED_FULL);
      break;
    case 'b':
      walkBackward(SPEED_FULL);
      break;
    case 'l':
      turnLeft(SPEED_TURN);
      break;
    case 'r':
      turnRight(SPEED_TURN);
      break;
    case 'F':
      walkForward(SPEED_HALF);
      break;
    case 'B':
      walkBackward(SPEED_HALF);
      break;
    case 'L':
      turnLeft(SPEED_HALF);
      break;
    case 'R':
      turnRight(SPEED_HALF);
      break;
    case 'q':
      spinLeft(SPEED_TURN);
      break;
    case 'e':
      spinRight(SPEED_TURN);
      break;
    case '0':
      stopWalking();
      break;

    // --------- 手臂控制 ---------
    case 'u':
      shoulderUp();
      break;
    case 'd':
      shoulderDown();
      break;
    case 'i':
      elbowUp();
      break;
    case 'k':
      elbowDown();
      break;
    case 'o':
      clawOpen();
      break;
    case 'p':
      clawClose();
      break;
    case 'h':
      armHome();
      break;

    // --------- 狀態查詢 ---------
    case 'v': {
      float v = readBatteryVoltage();
      Serial.print(F("Battery: "));
      Serial.print(v, 1);
      Serial.println(F("V"));
      // 也透過藍芽回傳給 App
      Serial1.print(v, 1);
      Serial1.print(F("V"));
      break;
    }

    default:
      break;
    }
  }
}
