from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LEG_REL = "esp32控制/02_LegController/02_LegController.ino"


def read(rel_path: str) -> str:
    return (ROOT / rel_path).read_text(encoding="utf-8")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def extract_function_body(text: str, function_name: str) -> str:
    signature_index = -1
    for return_type in ("void", "int", "bool"):
        signature = f"{return_type} {function_name}("
        signature_index = text.find(signature)
        if signature_index != -1:
            break

    if signature_index == -1:
        return ""

    open_brace_index = text.find("{", signature_index)
    if open_brace_index == -1:
        return ""

    depth = 0
    for index in range(open_brace_index, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[open_brace_index + 1 : index]

    return ""


def main() -> int:
    failures: list[str] = []
    leg_text = read(LEG_REL)

    for token in (
        "int leftMotorTrim = 0;",
        "int rightMotorTrim = 0;",
        "uint8_t activeLegCommand = CMD_LEG_STOP;",
        "const int TRIM_STEP = 5;",
        "const int TRIM_MIN = -80;",
        "const int TRIM_MAX = 80;",
        "int applyMotorTrim(int speed, int trim)",
        "void setLegMotors(int leftSpeed, int rightSpeed)",
        "void applyLegCommand(uint8_t cmd)",
        "void reapplyActiveMotion()",
        "void adjustLeftTrim(int delta)",
        "void adjustRightTrim(int delta)",
    ):
        require(token in leg_text, f"{LEG_REL} must contain {token}", failures)

    apply_trim = extract_function_body(leg_text, "applyMotorTrim")
    require(
        "abs(speed)" in apply_trim
        and "constrain" in apply_trim
        and "SPEED_MAX" in apply_trim
        and "speed > 0 ? 1 : -1" in apply_trim,
        f"{LEG_REL} applyMotorTrim() must preserve direction and constrain the trimmed magnitude",
        failures,
    )

    set_leg_motors = extract_function_body(leg_text, "setLegMotors")
    require(
        "applyMotorTrim(leftSpeed, leftMotorTrim)" in set_leg_motors
        and "applyMotorTrim(rightSpeed, rightMotorTrim)" in set_leg_motors
        and "setMotor(motorL_RPWM, motorL_LPWM" in set_leg_motors
        and "setMotor(motorR_RPWM, motorR_LPWM" in set_leg_motors,
        f"{LEG_REL} setLegMotors() must apply left/right trims before writing BTS7960 PWM",
        failures,
    )

    expected_motion_calls = {
        "forward": "setLegMotors(spd, spd)",
        "backward": "setLegMotors(-spd, -spd)",
        "spinLeft": "setLegMotors(-spd, spd)",
        "spinRight": "setLegMotors(spd, -spd)",
        "stopMotors": "setLegMotors(0, 0)",
    }
    for function_name, expected_call in expected_motion_calls.items():
        body = extract_function_body(leg_text, function_name)
        require(
            expected_call in body,
            f"{LEG_REL} {function_name}() must use {expected_call}",
            failures,
        )

    reapply_active_motion = extract_function_body(leg_text, "reapplyActiveMotion")
    require(
        "isMotionCommand(activeLegCommand)" in reapply_active_motion
        and "applyLegCommand(activeLegCommand)" in reapply_active_motion,
        f"{LEG_REL} reapplyActiveMotion() must immediately re-write PWM after trim changes while moving",
        failures,
    )

    adjust_left_trim = extract_function_body(leg_text, "adjustLeftTrim")
    adjust_right_trim = extract_function_body(leg_text, "adjustRightTrim")
    require(
        "reapplyActiveMotion()" in adjust_left_trim,
        f"{LEG_REL} adjustLeftTrim() must reapply the active motion command",
        failures,
    )
    require(
        "reapplyActiveMotion()" in adjust_right_trim,
        f"{LEG_REL} adjustRightTrim() must reapply the active motion command",
        failures,
    )

    execute_command = extract_function_body(leg_text, "executeCommand")
    require(
        "activeLegCommand = cmd" in execute_command
        and "applyLegCommand(cmd)" in execute_command,
        f"{LEG_REL} executeCommand() must track and dispatch the active leg command",
        failures,
    )

    loop_body = extract_function_body(leg_text, "loop")
    for token in (
        "case '[': adjustLeftTrim(-TRIM_STEP); break;",
        "case ']': adjustLeftTrim(TRIM_STEP); break;",
        "case '{': adjustRightTrim(-TRIM_STEP); break;",
        "case '}': adjustRightTrim(TRIM_STEP); break;",
    ):
        require(token in loop_body, f"{LEG_REL} loop() must handle Serial trim command {token}", failures)

    require("Left trim" in leg_text, f"{LEG_REL} status output must include left trim", failures)
    require("Right trim" in leg_text, f"{LEG_REL} status output must include right trim", failures)

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1

    print("PASS: leg motor trim controls are present and wired into motion commands")
    return 0


if __name__ == "__main__":
    sys.exit(main())
