/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — ESP-NOW 通訊協議定義
 *  共用標頭檔 (protocol.h)
 * =====================================================
 *
 *  本檔案定義所有 ESP-NOW 通訊的指令列舉、資料結構與常數。
 *  主控板與所有子控板共用，確保通訊協議一致。
 *
 *  使用方式：
 *    在各 .ino 檔案中加入 #include "../protocol.h"
 *
 *  注意：
 *    Arduino IDE 的 include 路徑從 sketch 目錄開始，
 *    因此使用 "../protocol.h" 可正確指向 esp32控制/ 根目錄。
 *    若 IDE 版本不支援，請將此檔案複製到各 sketch 目錄中。
 *
 *  架構（2026/05/05 更新）：
 *    C3 #1 — 腿部步行（L298N #1, JGB37-520 ×2）
 *    C3 #2 — 大圓盤旋轉 + z 升降（L298N #2, XD-25GA 370 + JGY370）
 *    C3 #3 — Servo 控制（MG996R 360° r齒條 + MG996R 180° θ + MG996R 180° 夾爪開合 + MG996R 180° 承物盒擋板）
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// =====================================================
//  速度常數
// =====================================================
const uint8_t FULL_SPEED = 200;
const uint8_t HALF_SPEED = 120;
const uint8_t SPEED_MIN  = 80;
const uint8_t SPEED_MAX  = 255;

// =====================================================
//  腿部指令（C3 #1 LegController）
//  L298N #1 → JGB37-520 ×2 差速轉向
// =====================================================
enum LegCommand {
  CMD_LEG_STOP = 0,
  CMD_FORWARD,       // 1 - 前進
  CMD_BACKWARD,      // 2 - 後退
  CMD_LEFT,          // 3 - 左轉
  CMD_RIGHT,         // 4 - 右轉
  CMD_SPIN_LEFT,     // 5 - 左旋
  CMD_SPIN_RIGHT,    // 6 - 右旋
};

typedef struct leg_now_message {
  uint8_t command;
  uint8_t speed;
} leg_now_message;

typedef struct leg_ack_message {
  uint8_t command;
  uint8_t speed;
  uint8_t status;
} leg_ack_message;

// =====================================================
//  大圓盤 + z 升降指令（C3 #2 TurntableZController）
//  L298N #2 Ch.A → XD-25GA 370 大圓盤旋轉
//  L298N #2 Ch.B → JGY370 z 升降（蝸桿自鎖）
// =====================================================
enum TurntableZCommand {
  CMD_TZ_STOP = 0,
  CMD_TURNTABLE_LEFT,   // 1 - 大圓盤左轉
  CMD_TURNTABLE_RIGHT,  // 2 - 大圓盤右轉
  CMD_Z_UP,             // 3 - z 上升
  CMD_Z_DOWN,           // 4 - z 下降
};

typedef struct turntable_z_now_message {
  uint8_t command;
  uint8_t speed;
} turntable_z_now_message;

// =====================================================
//  Servo / 夾爪 / 擋板指令（C3 #3 ServoClawController）
//  MG996R 360° → r 齒條伸縮
//  MG996R 180° → θ 旋轉
//  MG996R 180° → 夾爪開合
//  MG996R 180° → 承物盒擋板
// =====================================================
enum ServoClawCommand {
  CMD_SERVO_STOP = 0,
  CMD_R_EXTEND,        // 1 - r 齒條伸出（360° Servo 正轉）
  CMD_R_RETRACT,       // 2 - r 齒條縮回（360° Servo 反轉）
  CMD_THETA_POS,       // 3 - θ 旋轉（正方向）
  CMD_THETA_NEG,       // 4 - θ 旋轉（反方向）
  CMD_CLAW_OPEN,       // 5 - 夾爪張開
  CMD_CLAW_CLOSE,      // 6 - 夾爪閉合
  CMD_SERVO_HOME,      // 7 - 歸位（r 縮回 + 夾爪張開 + 擋板關閉）
  CMD_GATE_OPEN,       // 8 - 承物盒擋板打開
  CMD_GATE_CLOSE,      // 9 - 承物盒擋板關閉
};

typedef struct servo_claw_now_message {
  uint8_t command;
  uint8_t speed;     // 360° Servo: 速度控制, 180° Servo: 目標角度
} servo_claw_now_message;

#endif // PROTOCOL_H
