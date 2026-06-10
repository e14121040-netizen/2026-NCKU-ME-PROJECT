/*
 * =====================================================
 *  2026 NCKU 機械專題實作 — ESP-NOW / BLE 共用協議
 *  單一來源標頭檔 (protocol.h)
 * =====================================================
 *
 *  本檔案定義：
 *  - 各子控板的指令 enum
 *  - ESP-NOW message / ACK struct
 *  - 主控板與手機控制端共用的速度與停止別名
 *
 *  使用方式：
 *    在各 sketch 中加入 #include "../protocol.h"
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// =====================================================
//  速度常數
// =====================================================
const uint8_t FULL_SPEED = 200;
const uint8_t HALF_SPEED = 120;
const uint8_t LEG_DEFAULT_SPEED = 150;  // 4S 直供 12V 腿部馬達時的保守起始值
const uint8_t Z_DEFAULT_SPEED = 120;    // Z 升降使用較低 PWM，避免蝸桿機構動作過快
const uint8_t FIXED_SPIN_DEFAULT_SPEED = 120;  // 固定端 spin 使用較低 PWM，避免轉速過快
const uint8_t SPEED_MIN = 80;
const uint8_t SPEED_MAX = 255;

// =====================================================
//  手機 / Serial 分路停止別名
//  - '0' 仍保留給主控板全域急停
//  - '1' / '2' / '3' 供主控板本機 Serial 測試
// =====================================================
const char CMD_LEG_STOP_ONLY = '1';
const char CMD_ARM_STOP_ONLY = '2';
const char CMD_FIXED_STOP_ONLY = '3';

// Backward-compatible names used by older docs / app layouts.
const char CMD_TZ_STOP_ONLY = CMD_ARM_STOP_ONLY;
const char CMD_SERVO_STOP_ONLY = CMD_FIXED_STOP_ONLY;

// =====================================================
//  控制板識別碼（ACK / log 使用）
// =====================================================
enum ControllerId {
  CTRL_UNKNOWN = 0,
  CTRL_LEG = 1,
  CTRL_ROTATING_ARM = 2,
  CTRL_FIXED_STAGE = 3,

  // Backward-compatible ACK aliases.
  CTRL_TURNTABLE_Z = CTRL_ROTATING_ARM,
  CTRL_SERVO = CTRL_FIXED_STAGE,
};

const uint8_t ACK_STATUS_OK = 1;

// =====================================================
//  腿部指令（C3 #1 LegController）
// =====================================================
enum LegCommand {
  CMD_LEG_STOP = 0,
  CMD_FORWARD,       // 1 - 前進
  CMD_BACKWARD,      // 2 - 後退
  CMD_SPIN_LEFT,     // 3 - 原地左旋（左反右正）
  CMD_SPIN_RIGHT,    // 4 - 原地右旋（左正右反）
};

typedef struct leg_now_message {
  uint8_t command;
  uint8_t speed;
} leg_now_message;

// =====================================================
//  旋轉端上方機構指令（C3 #2 RotatingArmController）
// =====================================================
enum RotatingArmCommand {
  CMD_ARM_STOP = 0,
  CMD_ARM_R_EXTEND = 1,      // 1 - r 齒條伸出
  CMD_ARM_R_RETRACT = 2,     // 2 - r 齒條縮回
  CMD_ARM_THETA_POS = 3,     // 3 - θ 旋轉（正方向）
  CMD_ARM_THETA_NEG = 4,     // 4 - θ 旋轉（反方向）
  CMD_ARM_CLAW_OPEN = 5,     // 5 - 夾爪張開
  CMD_ARM_CLAW_CLOSE = 6,    // 6 - 夾爪閉合
  CMD_ARM_HOME = 7,          // 7 - 上方機構歸位（r 縮回 + θ 中央 + 夾爪張開）
  CMD_ARM_Z_UP = 8,          // 8 - z 上升
  CMD_ARM_Z_DOWN = 9,        // 9 - z 下降
  CMD_ARM_R_STOP = 10,       // 10 - 只停止 r 齒條

  // Backward-compatible command aliases.
  CMD_TZ_STOP = CMD_ARM_STOP,
  CMD_Z_UP = CMD_ARM_Z_UP,
  CMD_Z_DOWN = CMD_ARM_Z_DOWN,
  CMD_SERVO_STOP = CMD_ARM_STOP,
  CMD_R_EXTEND = CMD_ARM_R_EXTEND,
  CMD_R_RETRACT = CMD_ARM_R_RETRACT,
  CMD_THETA_POS = CMD_ARM_THETA_POS,
  CMD_THETA_NEG = CMD_ARM_THETA_NEG,
  CMD_CLAW_OPEN = CMD_ARM_CLAW_OPEN,
  CMD_CLAW_CLOSE = CMD_ARM_CLAW_CLOSE,
  CMD_SERVO_HOME = CMD_ARM_HOME,
  CMD_R_STOP = CMD_ARM_R_STOP,
};

typedef struct rotating_arm_now_message {
  uint8_t command;
  uint8_t speed;
} rotating_arm_now_message;

typedef rotating_arm_now_message turntable_z_now_message;

// =====================================================
//  固定端暫存盒 / 旋轉馬達指令（C3 #3 FixedStageController）
// =====================================================
enum FixedStageCommand {
  CMD_FIXED_STOP = 0,
  CMD_FIXED_SPIN_LEFT = 1,    // 1 - 固定端旋轉馬達左轉
  CMD_FIXED_SPIN_RIGHT = 2,   // 2 - 固定端旋轉馬達右轉
  CMD_FIXED_GATE_OPEN = 3,    // 3 - 承物盒擋板打開
  CMD_FIXED_GATE_CLOSE = 4,   // 4 - 承物盒擋板關閉

  // Backward-compatible command aliases.
  CMD_TURNTABLE_LEFT = CMD_FIXED_SPIN_LEFT,
  CMD_TURNTABLE_RIGHT = CMD_FIXED_SPIN_RIGHT,
  CMD_GATE_OPEN = CMD_FIXED_GATE_OPEN,
  CMD_GATE_CLOSE = CMD_FIXED_GATE_CLOSE,
};

typedef struct fixed_stage_now_message {
  uint8_t command;
  uint8_t speed;
} fixed_stage_now_message;

typedef fixed_stage_now_message servo_claw_now_message;

// =====================================================
//  共用 ACK 格式
// =====================================================
typedef struct ack_message {
  uint8_t controller_id;
  uint8_t command;
  uint8_t speed;
  uint8_t status;
} ack_message;

#endif  // PROTOCOL_H
