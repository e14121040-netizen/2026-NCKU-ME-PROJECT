# Gate Position Hold Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make the fixed-stage gate use 180-degree positional servo angles with smaller travel and keep the closed gate held against part pressure.

**Architecture:** Fixed-stage gate control moves from timed continuous-style commands to explicit positional angles. `GATEOPEN` writes a reduced open angle, while `GATECLOSE` writes a reduced close angle and keeps PWM attached so the servo holds position. Verification rejects the old speed/time gate behavior and documents the new field test expectation.

**Tech Stack:** Arduino C++ sketch for ESP32-C3, `ESP32Servo`, Python verifier script, `arduino-cli compile`.

---

### Task 1: Update Gate Verifier Expectations

**Files:**
- Modify: `tools/verify_esp32_mobile_control_consistency.py`

**Step 1: Write the failing verification requirement**

Update the fixed-stage checks to require:

```python
extract_int_constant(fixed_text, "GATE_OPEN_ANGLE") == 160
extract_int_constant(fixed_text, "GATE_CLOSE_ANGLE") == 20
"GATE_OPEN_SPEED" not in fixed_text
"GATE_CLOSE_SPEED" not in fixed_text
"GATE_RUN_TIME_MS" not in fixed_text
"gateServo.write(GATE_CLOSE_ANGLE)" in fixed_gate_close
"gateStop()" not in fixed_check_gate_timeout
```

Also extract:

```python
fixed_gate_close = extract_function_body(fixed_text, "gateClose")
fixed_check_gate_timeout = extract_function_body(fixed_text, "checkGateTimeout")
```

**Step 2: Run verifier to verify it fails**

Run:

```bash
python3 tools/verify_esp32_mobile_control_consistency.py
```

Expected: FAIL because `04_FixedStageController.ino` still defines `GATE_OPEN_SPEED`, `GATE_CLOSE_SPEED`, and `GATE_RUN_TIME_MS`, and does not define angle constants.

**Step 3: Commit verifier change**

After the expected failure is confirmed:

```bash
git add tools/verify_esp32_mobile_control_consistency.py
git commit -m "test: require gate position hold behavior"
```

### Task 2: Implement Gate Position Hold

**Files:**
- Modify: `esp32控制/04_FixedStageController/04_FixedStageController.ino`

**Step 1: Replace gate constants**

Change:

```cpp
const int GATE_STOP        = 90;
const int GATE_OPEN_SPEED  = 180;
const int GATE_CLOSE_SPEED = 0;
const unsigned long GATE_RUN_TIME_MS = 500;

int currentGateSpeed = GATE_STOP;
```

To:

```cpp
const int GATE_OPEN_ANGLE  = 160;
const int GATE_CLOSE_ANGLE = 20;
const unsigned long GATE_OPEN_HOLD_MS = 700;

int currentGateAngle = GATE_CLOSE_ANGLE;
```

**Step 2: Replace timed motion state**

Change gate state so it tracks only the open hold timer:

```cpp
unsigned long gateOpenHoldUntilMs = 0;
bool gateOpenHolding = false;
```

Remove the old `gateStartTime` and `gateMoving` behavior.

**Step 3: Implement positional open/close**

Use:

```cpp
void gateStop() {
  detachGateServoIfNeeded();
  gateOpenHolding = false;
  Serial.println("Gate STOP");
}

void gateOpen() {
  attachGateServoIfNeeded();
  currentGateAngle = GATE_OPEN_ANGLE;
  gateServo.write(GATE_OPEN_ANGLE);
  gateOpenHolding = true;
  gateOpenHoldUntilMs = millis() + GATE_OPEN_HOLD_MS;
  Serial.print("Gate OPEN -> angle ");
  Serial.println(currentGateAngle);
}

void gateClose() {
  attachGateServoIfNeeded();
  currentGateAngle = GATE_CLOSE_ANGLE;
  gateServo.write(GATE_CLOSE_ANGLE);
  gateOpenHolding = false;
  Serial.print("Gate CLOSE hold -> angle ");
  Serial.println(currentGateAngle);
}

void checkGateTimeout() {
  if (gateOpenHolding && millis() >= gateOpenHoldUntilMs) {
    gateOpenHolding = false;
    detachGateServoIfNeeded();
    Serial.println("Gate open hold complete -> PWM disabled");
  }
}
```

**Step 4: Update status output**

Change serial status text from speed/moving to:

```cpp
Serial.print("  Gate PWM attached: ");
Serial.println(gateServoAttached ? "YES" : "NO");
Serial.print("  Gate angle: ");
Serial.println(currentGateAngle);
Serial.print("  Gate open hold: ");
Serial.println(gateOpenHolding ? "YES" : "NO");
```

**Step 5: Run verifier to verify it passes**

Run:

```bash
python3 tools/verify_esp32_mobile_control_consistency.py
```

Expected: PASS with no failures.

**Step 6: Compile fixed-stage sketch**

Run:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c3 esp32控制/04_FixedStageController
```

Expected: successful compile.

**Step 7: Commit implementation**

```bash
git add esp32控制/04_FixedStageController/04_FixedStageController.ino
git commit -m "fix: hold fixed gate close position"
```

### Task 3: Update Field Test Documentation

**Files:**
- Modify: `開發步驟指南.md`

**Step 1: Update fixed-stage gate verification checklist**

Change the fixed-stage gate verification from timed PWM stop only to:

```markdown
- [ ] Gate Open / Close 正常，Open 後短暫保持再釋放 PWM
- [ ] Gate Close 後 PWM 維持輸出，零件推擠時不會被打開
```

**Step 2: Run verifier**

Run:

```bash
python3 tools/verify_esp32_mobile_control_consistency.py
```

Expected: PASS.

**Step 3: Commit docs**

```bash
git add 開發步驟指南.md
git commit -m "docs: document gate close hold test"
```
