from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(rel_path: str) -> str:
    return (ROOT / rel_path).read_text(encoding="utf-8")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def extract_function_body(text: str, function_name: str) -> str:
    signature = f"void {function_name}("
    signature_index = text.find(signature)
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


def extract_int_constant(text: str, constant_name: str) -> int | None:
    pattern = rf"\b(?:const\s+)?(?:uint8_t|int|unsigned\s+long)\s+{re.escape(constant_name)}\s*=\s*(\d+)\s*;"
    match = re.search(pattern, text)
    if match is None:
        return None
    return int(match.group(1))


def main() -> int:
    failures: list[str] = []

    protocol_rel = "esp32控制/protocol.h"
    protocol_text = read(protocol_rel)
    for token in (
        "enum LegCommand",
        "enum RotatingArmCommand",
        "enum FixedStageCommand",
        "typedef struct leg_now_message",
        "typedef struct rotating_arm_now_message",
        "typedef struct fixed_stage_now_message",
        "typedef struct ack_message",
        "CTRL_ROTATING_ARM",
        "CTRL_FIXED_STAGE",
        "CMD_ARM_STOP_ONLY",
        "CMD_FIXED_STOP_ONLY",
        "CMD_ARM_R_STOP",
        "CMD_FIXED_GATE_OPEN",
        "CMD_FIXED_GATE_CLOSE",
    ):
        require(token in protocol_text, f"{protocol_rel} must define {token}", failures)

    require(
        extract_int_constant(protocol_text, "Z_DEFAULT_SPEED") == 120,
        f"{protocol_rel} must define Z_DEFAULT_SPEED as 120",
        failures,
    )

    sketch_rels = [
        "esp32控制/01_MainController/01_MainController.ino",
        "esp32控制/02_LegController/02_LegController.ino",
        "esp32控制/03_RotatingArmController/03_RotatingArmController.ino",
        "esp32控制/04_FixedStageController/04_FixedStageController.ino",
    ]
    duplicate_tokens = [
        "enum LegCommand",
        "enum RotatingArmCommand",
        "enum FixedStageCommand",
        "enum TurntableZCommand",
        "enum ServoClawCommand",
        "typedef struct leg_ack_message",
        "typedef struct ack_message",
    ]
    for rel in sketch_rels:
        path = ROOT / rel
        require(path.exists(), f"{rel} must exist", failures)
        if not path.exists():
            continue
        text = read(rel)
        require('#include "../protocol.h"' in text, f"{rel} must include ../protocol.h", failures)
        for token in duplicate_tokens:
            require(token not in text, f"{rel} must not redefine {token}", failures)

    leg_rel = "esp32控制/02_LegController/02_LegController.ino"
    leg_text = read(leg_rel)
    require("const int SPEED_MIN" not in leg_text, f"{leg_rel} must not shadow shared SPEED_MIN", failures)
    require("const int SPEED_MAX" not in leg_text, f"{leg_rel} must not shadow shared SPEED_MAX", failures)
    require("#include <esp_wifi.h>" in leg_text, f"{leg_rel} must include esp_wifi.h for fixed ESPNOW channel", failures)
    require("esp_wifi_set_channel" in leg_text, f"{leg_rel} must set the ESP-NOW WiFi channel", failures)
    require("COMMAND_TIMEOUT_MS" in leg_text, f"{leg_rel} must define a remote command watchdog timeout", failures)
    require("checkCommandTimeout" in leg_text, f"{leg_rel} must stop motors when remote commands time out", failures)

    arm_rel = "esp32控制/03_RotatingArmController/03_RotatingArmController.ino"
    if (ROOT / arm_rel).exists():
        arm_text = read(arm_rel)
        arm_setup = extract_function_body(arm_text, "setup")
        arm_execute = extract_function_body(arm_text, "executeCommand")
        arm_all_stop = extract_function_body(arm_text, "allStop")
        arm_loop = extract_function_body(arm_text, "loop")
        arm_theta_pos = extract_function_body(arm_text, "thetaPos")
        arm_theta_neg = extract_function_body(arm_text, "thetaNeg")
        arm_claw_open = extract_function_body(arm_text, "clawOpen")
        arm_claw_close = extract_function_body(arm_text, "clawClose")
        arm_home = extract_function_body(arm_text, "startHome")
        arm_schedule_standard_servo_detach = extract_function_body(arm_text, "scheduleStandardServoDetach")
        arm_check_standard_servo_detach = extract_function_body(arm_text, "checkStandardServoDetachTimeout")

        for token in (
            "#include <ESP32Servo.h>",
            "rotating_arm_now_message incomingMsg",
            "const int servoR_Pin",
            "const int servoTheta_Pin",
            "const int servoClaw_Pin",
            "const int motorZ_RPWM",
            "const int motorZ_LPWM",
            "pwmCh_Z_RPWM  = 4",
            "pwmCh_Z_LPWM  = 5",
            "attachRServoIfNeeded",
            "attachThetaServoIfNeeded",
            "attachClawServoIfNeeded",
            "zUp()",
            "zDown()",
            "checkZTimeout",
            "checkRTimeout",
            "scheduleStandardServoDetach",
            "checkStandardServoDetachTimeout",
        ):
            require(token in arm_text, f"{arm_rel} must contain {token}", failures)

        require(
            extract_int_constant(arm_text, "R_EXTEND") == 0,
            f"{arm_rel} must set R_EXTEND to 0 for the current rack servo direction",
            failures,
        )
        require(
            extract_int_constant(arm_text, "R_RETRACT") == 180,
            f"{arm_rel} must set R_RETRACT to 180 for the current rack servo direction",
            failures,
        )
        require(
            "int dutyCycle = Z_DEFAULT_SPEED;" in arm_text,
            f"{arm_rel} default Z dutyCycle must use Z_DEFAULT_SPEED",
            failures,
        )
        require(
            extract_int_constant(arm_text, "STANDARD_SERVO_HOLD_MS") == 700,
            f"{arm_rel} must hold theta/claw PWM for 700ms after position commands",
            failures,
        )
        require(
            "standardServoDetachAtMs = millis() + STANDARD_SERVO_HOLD_MS" in arm_schedule_standard_servo_detach,
            f"{arm_rel} scheduleStandardServoDetach() must delay theta/claw detach",
            failures,
        )
        require(
            "detachThetaServoIfNeeded()" in arm_check_standard_servo_detach
            and "detachClawServoIfNeeded()" in arm_check_standard_servo_detach
            and "standardServoDetachAtMs = 0" in arm_check_standard_servo_detach,
            f"{arm_rel} checkStandardServoDetachTimeout() must detach theta/claw after the hold window",
            failures,
        )

        for function_name, body in (
            ("thetaPos()", arm_theta_pos),
            ("thetaNeg()", arm_theta_neg),
            ("clawOpen()", arm_claw_open),
            ("clawClose()", arm_claw_close),
            ("startHome()", arm_home),
        ):
            require(
                "scheduleStandardServoDetach()" in body,
                f"{arm_rel} {function_name} must keep theta/claw PWM enabled briefly after motion",
                failures,
            )

        for forbidden in (
            "motorTurntable",
            "CMD_FIXED_SPIN_LEFT",
            "CMD_FIXED_SPIN_RIGHT",
            "servoGate",
        ):
            require(forbidden not in arm_text, f"{arm_rel} must not control fixed-stage {forbidden}", failures)

        require(arm_setup, f"{arm_rel} must define setup()", failures)
        for forbidden in (
            "servoR.attach",
            "servoTheta.attach",
            "servoClaw.attach",
            "rStop()",
            "clawOpen()",
            "servoTheta.write",
        ):
            require(forbidden not in arm_setup, f"{arm_rel} setup() must not command {forbidden} during boot", failures)

        require(arm_execute, f"{arm_rel} must define executeCommand()", failures)
        for token in (
            "case CMD_ARM_R_EXTEND:",
            "case CMD_ARM_R_RETRACT:",
            "case CMD_ARM_R_STOP:",
            "case CMD_ARM_THETA_POS:",
            "case CMD_ARM_THETA_NEG:",
            "case CMD_ARM_CLAW_OPEN:",
            "case CMD_ARM_CLAW_CLOSE:",
            "case CMD_ARM_Z_UP:",
            "case CMD_ARM_Z_DOWN:",
            "case CMD_ARM_HOME:",
        ):
            require(token in arm_execute, f"{arm_rel} executeCommand() must route {token}", failures)

        require(
            "checkStandardServoDetachTimeout();" in arm_loop,
            f"{arm_rel} loop() must service delayed theta/claw detach",
            failures,
        )
        require(
            "rStop()" in arm_all_stop and "zStop()" in arm_all_stop,
            f"{arm_rel} allStop() must immediately stop rack and Z motion",
            failures,
        )
        require(
            "detachAllServoOutputs()" not in arm_all_stop
            and "detachThetaServoIfNeeded()" not in arm_all_stop
            and "detachClawServoIfNeeded()" not in arm_all_stop,
            f"{arm_rel} ARMSTOP must not immediately detach theta/claw standard servo PWM",
            failures,
        )

    fixed_rel = "esp32控制/04_FixedStageController/04_FixedStageController.ino"
    if (ROOT / fixed_rel).exists():
        fixed_text = read(fixed_rel)
        fixed_setup = extract_function_body(fixed_text, "setup")
        fixed_execute = extract_function_body(fixed_text, "executeCommand")
        fixed_all_stop = extract_function_body(fixed_text, "allStop")

        for token in (
            "#include <ESP32Servo.h>",
            "fixed_stage_now_message incomingMsg",
            "const int motorSpin_RPWM",
            "const int motorSpin_LPWM",
            "const int servoGate_Pin",
            "pwmCh_Spin_RPWM  = 4",
            "pwmCh_Spin_LPWM  = 5",
            "spinLeft()",
            "spinRight()",
            "spinStop()",
            "attachGateServoIfNeeded",
            "checkSpinTimeout",
            "checkGateTimeout",
            "GATE_OPEN_SPEED",
            "GATE_CLOSE_SPEED",
        ):
            require(token in fixed_text, f"{fixed_rel} must contain {token}", failures)

        for forbidden in (
            "servoR",
            "servoTheta",
            "servoClaw",
            "CMD_ARM_R_EXTEND",
            "CMD_ARM_Z_UP",
        ):
            require(forbidden not in fixed_text, f"{fixed_rel} must not control rotating-arm {forbidden}", failures)

        require(fixed_setup, f"{fixed_rel} must define setup()", failures)
        require(
            "servoGate.attach" not in fixed_setup and "gateClose()" not in fixed_setup,
            f"{fixed_rel} setup() must not command gate servo during boot",
            failures,
        )

        require(fixed_execute, f"{fixed_rel} must define executeCommand()", failures)
        for token in (
            "case CMD_FIXED_SPIN_LEFT:",
            "case CMD_FIXED_SPIN_RIGHT:",
            "case CMD_FIXED_GATE_OPEN:",
            "case CMD_FIXED_GATE_CLOSE:",
        ):
            require(token in fixed_execute, f"{fixed_rel} executeCommand() must route {token}", failures)

        require(
            "spinStop()" in fixed_all_stop and "gateStop()" in fixed_all_stop,
            f"{fixed_rel} allStop() must stop fixed spin motor and gate servo",
            failures,
        )

    main_rel = "esp32控制/01_MainController/01_MainController.ino"
    main_text = read(main_rel)
    for token in (
        "LEGSTOP",
        "ARMSTOP",
        "FIXEDSTOP",
        "TZSTOP",
        "SERVOSTOP",
        "RSTOP",
        "SPD:",
        "GATEOPEN",
        "GATECLOSE",
        "COMMAND_KEEPALIVE_INTERVAL_MS",
        "normalizeBleCommandText",
        "normalizeThetaCommandText",
        "case 'I':",
        "case 'K':",
        "uint8_t rotatingArm_Address[] = {0x10, 0xB4, 0x1D, 0x1C, 0xD1, 0x28}",
        "uint8_t fixedStage_Address[] = {0x58, 0x8C, 0x81, 0xA1, 0x30, 0xD0}",
    ):
        require(token in main_text, f"{main_rel} must contain {token}", failures)

    for token in (
        "queueRotatingArmCommand(CMD_ARM_Z_UP",
        "queueRotatingArmCommand(CMD_ARM_Z_DOWN",
        "queueRotatingArmCommand(CMD_ARM_R_EXTEND",
        "queueRotatingArmCommand(CMD_ARM_R_RETRACT",
        "queueRotatingArmCommand(CMD_ARM_R_STOP",
        "queueRotatingArmCommand(CMD_ARM_THETA_POS",
        "queueRotatingArmCommand(CMD_ARM_THETA_NEG",
        "queueRotatingArmCommand(CMD_ARM_CLAW_OPEN",
        "queueRotatingArmCommand(CMD_ARM_CLAW_CLOSE",
        "queueFixedStageCommand(CMD_FIXED_SPIN_LEFT",
        "queueFixedStageCommand(CMD_FIXED_SPIN_RIGHT",
        "queueFixedStageCommand(CMD_FIXED_GATE_OPEN",
        "queueFixedStageCommand(CMD_FIXED_GATE_CLOSE",
    ):
        require(token in main_text, f"{main_rel} must route {token}", failures)

    for token in (
        "queueRotatingArmCommand(CMD_ARM_Z_UP, Z_DEFAULT_SPEED)",
        "queueRotatingArmCommand(CMD_ARM_Z_DOWN, Z_DEFAULT_SPEED)",
    ):
        require(token in main_text, f"{main_rel} must route Z movement with {token}", failures)

    for token in (
        "queueRotatingArmCommand(CMD_ARM_Z_UP, FULL_SPEED)",
        "queueRotatingArmCommand(CMD_ARM_Z_DOWN, FULL_SPEED)",
    ):
        require(token not in main_text, f"{main_rel} must not route Z movement with {token}", failures)

    require(
        'normalizedCmd == "SERVOSTOP"' in main_text and "queueRotatingArmCommand(CMD_ARM_STOP" in main_text,
        f"{main_rel} must keep SERVOSTOP as a rotating-arm servo stop alias",
        failures,
    )
    require(
        'normalizedCmd == "FIXEDSTOP"' in main_text and "queueFixedStageCommand(CMD_FIXED_STOP" in main_text,
        f"{main_rel} must support FIXEDSTOP for C3 #3",
        failures,
    )
    require(
        "queueRotatingArmCommand(CMD_ARM_HOME" in main_text
        and "queueFixedStageCommand(CMD_FIXED_GATE_CLOSE" in main_text,
        f"{main_rel} HOME must route arm home and fixed gate close",
        failures,
    )

    doc_rels = [
        "README.md",
        "開發步驟指南.md",
        "esp32控制/README.md",
        "取物機器人(八足Jansen)/app_inventor/App指令對照表.md",
        "取物機器人(八足Jansen)/app_inventor/AppInventor開發指南.md",
        "取物機器人(八足Jansen)/電控/電控組裝SOP.md",
    ]
    combined = "\n".join(read(rel) for rel in doc_rels)
    require(
        "BLE Controller – Arduino ESP32" in combined,
        "formal docs must mention BLE Controller – Arduino ESP32",
        failures,
    )
    require(
        "ESP32_MainController" in combined,
        "formal docs must mention the BLE device name ESP32_MainController",
        failures,
    )

    official_app_doc_rel = "取物機器人(八足Jansen)/app_inventor/App指令對照表.md"
    official_app_doc = read(official_app_doc_rel)
    for token in ("LEGSTOP", "ARMSTOP", "FIXEDSTOP", "TZSTOP", "SERVOSTOP", "RSTOP", "STOP"):
        require(token in official_app_doc, f"{official_app_doc_rel} must document {token}", failures)
    require("TouchUp" not in official_app_doc, f"{official_app_doc_rel} must not teach TouchUp stop flow", failures)
    require("PickupRobot" not in official_app_doc, f"{official_app_doc_rel} must not use PickupRobot", failures)
    require(
        "Classic BT" not in official_app_doc,
        f"{official_app_doc_rel} must not present Classic BT as official flow",
        failures,
    )

    guide_rel = "取物機器人(八足Jansen)/app_inventor/AppInventor開發指南.md"
    guide_text = read(guide_rel)
    require(
        "歷史" in guide_text or "備用" in guide_text,
        f"{guide_rel} must clearly mark App Inventor as historical/backup guidance",
        failures,
    )
    require("PickupRobot" not in guide_text, f"{guide_rel} must not instruct users to pair PickupRobot", failures)
    require("TouchUp" not in guide_text, f"{guide_rel} must not teach TouchUp stop flow", failures)

    ble_guide_rel = "取物機器人(八足Jansen)/app_inventor/BLE_Controller控制指南.md"
    require((ROOT / ble_guide_rel).exists(), f"{ble_guide_rel} must exist", failures)
    if (ROOT / ble_guide_rel).exists():
        ble_guide_text = read(ble_guide_rel)
        for token in (
            "BLE Controller – Arduino ESP32",
            "LEGSTOP",
            "ARMSTOP",
            "FIXEDSTOP",
            "TZSTOP",
            "SERVOSTOP",
            "RSTOP",
            "STOP",
        ):
            require(token in ble_guide_text, f"{ble_guide_rel} must document {token}", failures)

    root_readme = read("README.md")
    pickup_readme = read("取物機器人(八足Jansen)/README.md")
    wiring_rel = "取物機器人(八足Jansen)/電控/接線圖.md"
    wiring_text = read(wiring_rel)
    power_rel = "取物機器人(八足Jansen)/電控/電源預算.md"
    power_text = read(power_rel)
    sop_rel = "取物機器人(八足Jansen)/電控/電控組裝SOP.md"
    sop_text = read(sop_rel)
    bom_text = read("BOM.md")
    pickup_bom_text = read("取物機器人(八足Jansen)/BOM.md")
    esp32_readme = read("esp32控制/README.md")

    require("原地旋轉轉向" in root_readme, "README.md must describe walking turn mode as spin turn", failures)
    require("差速轉向" not in pickup_readme, "pickup README must not describe the current leg control as differential turn", failures)
    for token in (
        "03_RotatingArmController",
        "04_FixedStageController",
        "C3 #2 旋轉端",
        "C3 #3 固定端",
    ):
        require(token in esp32_readme, f"esp32控制/README.md must document {token}", failures)
    for token in (
        "C3 #2 RotatingArm",
        "C3 #3 FixedStage",
        "旋轉中心",
        "鬆弛線圈",
        "Servo ×3",
    ):
        require(token in wiring_text, f"{wiring_rel} must document {token}", failures)
    require("VCC" in wiring_text and "3.3V" in wiring_text, f"{wiring_rel} must document BTS7960 logic VCC wiring", failures)
    require("BMS" in bom_text and "保險絲座" in bom_text and "端子台" in bom_text, "BOM.md must include power safety parts", failures)
    require("BMS" in pickup_bom_text and "保險絲座" in pickup_bom_text and "端子台" in pickup_bom_text, "pickup BOM must include power safety parts", failures)
    require("2S 18650" in power_text and "不建議" in power_text, f"{power_rel} must document XL4015 two-cell feasibility limits", failures)
    require("2S 18650" in sop_text and "共地" in sop_text, f"{sop_rel} must document separate servo battery wiring and common ground", failures)

    if failures:
        print("ESP32 mobile control consistency check FAILED:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("ESP32 mobile control consistency check PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
