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
//  手臂 r/θ 指令（C3 #2 GripperController）
// =====================================================
enum GripperCommand {
  CMD_GRIPPER_STOP = 0,
  CMD_ARM_EXTEND,    // 1 - r+ 手臂伸出
  CMD_ARM_RETRACT,   // 2 - r- 手臂縮回
  CMD_ARM_LEFT,      // 3 - θ- 手臂左轉
  CMD_ARM_RIGHT,     // 4 - θ+ 手臂右轉
};

typedef struct gripper_now_message {
  uint8_t command;
  uint8_t speed;
} gripper_now_message;

// =====================================================
//  z/夾爪指令（C3 #3 ZClawController）
// =====================================================
enum ZClawCommand {
  CMD_ZCLAW_STOP = 0,
  CMD_Z_UP,          // 1 - z 上升
  CMD_Z_DOWN,        // 2 - z 下降
  CMD_CLAW_OPEN,     // 3 - 夾爪張開（伺服）
  CMD_CLAW_CLOSE,    // 4 - 夾爪閉合（伺服）
  CMD_LID_OPEN,      // 5 - 蓋板開（伺服）
  CMD_LID_CLOSE,     // 6 - 蓋板關（伺服）
  CMD_HOME,          // 7 - 歸位
};

typedef struct zclaw_now_message {
  uint8_t command;
  uint8_t speed;     // DC 馬達速度 or 伺服角度
} zclaw_now_message;

#endif // PROTOCOL_H
