# Rotating Arm Fixed Stage Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Move rotating upper-arm actuators to C3 #2 and fixed-stage actuators to C3 #3 to prevent wire twisting.

**Architecture:** C3 #2 becomes `RotatingArmController` and owns upper-arm servos plus Z motor. C3 #3 becomes `FixedStageController` and owns the fixed spin motor plus gate servo. The main ESP32 keeps the BLE command surface stable and only changes ESP-NOW routing.

**Tech Stack:** Arduino ESP32 core, ESP-NOW, BLE UART, `ESP32Servo`, BTS7960 PWM, Python consistency checker.

---

### Task 1: Add Failing Consistency Checks

**Files:**
- Modify: `tools/verify_esp32_mobile_control_consistency.py`

**Step 1: Write the failing test**

Add requirements for:

- `enum RotatingArmCommand`
- `enum FixedStageCommand`
- `rotating_arm_now_message`
- `fixed_stage_now_message`
- C3 #2 sketch path `esp32控制/03_RotatingArmController/03_RotatingArmController.ino`
- C3 #3 sketch path `esp32控制/04_FixedStageController/04_FixedStageController.ino`
- Main command routing tokens showing arm commands go through `queueRotatingArmCommand` and fixed commands go through `queueFixedStageCommand`

**Step 2: Run test to verify it fails**

Run: `python3 tools/verify_esp32_mobile_control_consistency.py`

Expected: FAIL because the current code still uses TurntableZ / ServoClaw split.

### Task 2: Update Protocol And Main Routing

**Files:**
- Modify: `esp32控制/protocol.h`
- Modify: `esp32控制/01_MainController/01_MainController.ino`

**Step 1: Implement protocol**

Define `RotatingArmCommand`, `FixedStageCommand`, `rotating_arm_now_message`, and `fixed_stage_now_message`. Keep compatibility aliases only where they reduce app churn.

**Step 2: Implement main routing**

Route:

- Arm/Z/gripper commands to C3 #2.
- Spin/Gate commands to C3 #3.
- `HOME` to both C3 #2 arm home and C3 #3 gate close.
- `STOP` to all three child controllers.

**Step 3: Run consistency check**

Run: `python3 tools/verify_esp32_mobile_control_consistency.py`

Expected: still FAIL until child sketches and docs are updated.

### Task 3: Replace Child Controller Sketches

**Files:**
- Rename: `esp32控制/03_TurntableZController/03_TurntableZController.ino` -> `esp32控制/03_RotatingArmController/03_RotatingArmController.ino`
- Rename: `esp32控制/04_ServoClawController/04_ServoClawController.ino` -> `esp32控制/04_FixedStageController/04_FixedStageController.ino`

**Step 1: Implement C3 #2 RotatingArm**

Combine lazy servo control for rack/theta/claw with BTS7960 Z motor control. Use high LEDC channels for Z PWM to avoid servo channel allocation conflicts.

**Step 2: Implement C3 #3 FixedStage**

Combine BTS7960 spin motor control with lazy timed gate servo control.

**Step 3: Run consistency check**

Run: `python3 tools/verify_esp32_mobile_control_consistency.py`

Expected: FAIL only on documentation if docs are not updated yet.

### Task 4: Update Documentation

**Files:**
- Modify: `esp32控制/README.md`
- Modify: `README.md`
- Modify: `取物機器人(八足Jansen)/電控/接線圖.md`
- Modify as needed: `開發步驟指南.md`

**Step 1: Update architecture and upload instructions**

Rename the controller descriptions to C3 #2 `RotatingArm` and C3 #3 `FixedStage`.

**Step 2: Update wiring**

Document C3 #2 rotating upper-arm wiring and C3 #3 fixed lower-stage wiring, including center slack-loop guidance.

**Step 3: Run consistency check**

Run: `python3 tools/verify_esp32_mobile_control_consistency.py`

Expected: PASS.

### Task 5: Compile Or Report Tooling Gap

**Files:**
- Verify only.

**Step 1: Try Arduino compile**

Run:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app esp32控制/01_MainController/
arduino-cli compile --fqbn esp32:esp32:esp32c3 esp32控制/03_RotatingArmController/
arduino-cli compile --fqbn esp32:esp32:esp32c3 esp32控制/04_FixedStageController/
```

**Step 2: Report result**

If `arduino-cli` or boards/libraries are missing, report that compile verification could not be completed and include the exact error.
