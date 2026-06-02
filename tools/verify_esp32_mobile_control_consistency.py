from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(rel_path: str) -> str:
    return (ROOT / rel_path).read_text(encoding="utf-8")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def extract_function_body(text: str, function_name: str) -> str:
    signature = f"void {function_name}()"
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


def main() -> int:
    failures: list[str] = []

    protocol_rel = "esp32控制/protocol.h"
    protocol_text = read(protocol_rel)
    require("enum LegCommand" in protocol_text, f"{protocol_rel} must define LegCommand", failures)
    require(
        "enum TurntableZCommand" in protocol_text,
        f"{protocol_rel} must define TurntableZCommand",
        failures,
    )
    require(
        "enum ServoClawCommand" in protocol_text,
        f"{protocol_rel} must define ServoClawCommand",
        failures,
    )
    require(
        "typedef struct ack_message" in protocol_text,
        f"{protocol_rel} must define shared ack_message",
        failures,
    )
    require("CTRL_LEG" in protocol_text, f"{protocol_rel} must define controller ids", failures)
    require("CMD_LEG_STOP_ONLY" in protocol_text, f"{protocol_rel} must define split stop commands", failures)
    require("CMD_TZ_STOP_ONLY" in protocol_text, f"{protocol_rel} must define split stop commands", failures)
    require(
        "CMD_SERVO_STOP_ONLY" in protocol_text,
        f"{protocol_rel} must define split stop commands",
        failures,
    )

    sketch_rels = [
        "esp32控制/01_MainController/01_MainController.ino",
        "esp32控制/02_LegController/02_LegController.ino",
        "esp32控制/03_TurntableZController/03_TurntableZController.ino",
        "esp32控制/04_ServoClawController/04_ServoClawController.ino",
    ]
    duplicate_tokens = [
        "enum LegCommand",
        "enum TurntableZCommand",
        "enum ServoClawCommand",
        "typedef struct leg_ack_message",
        "typedef struct ack_message",
    ]
    for rel in sketch_rels:
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

    tz_rel = "esp32控制/03_TurntableZController/03_TurntableZController.ino"
    tz_text = read(tz_rel)
    require("const int SPEED_MIN" not in tz_text, f"{tz_rel} must not shadow shared SPEED_MIN", failures)
    require("const int SPEED_MAX" not in tz_text, f"{tz_rel} must not shadow shared SPEED_MAX", failures)

    servo_rel = "esp32控制/04_ServoClawController/04_ServoClawController.ino"
    servo_text = read(servo_rel)
    require("#include <esp_wifi.h>" in servo_text, f"{servo_rel} must include esp_wifi.h for fixed ESPNOW channel", failures)
    require("esp_wifi_set_channel" in servo_text, f"{servo_rel} must set the ESP-NOW WiFi channel", failures)
    servo_setup = extract_function_body(servo_text, "setup")
    require(servo_setup, f"{servo_rel} must define setup()", failures)
    require(
        "servoR.attach" not in servo_setup,
        f"{servo_rel} setup() must not attach GPIO 0 r servo during boot",
        failures,
    )
    require(
        "servoClaw.attach" not in servo_setup,
        f"{servo_rel} setup() must not attach GPIO 4 claw servo during boot",
        failures,
    )
    require(
        "servoGate.attach" not in servo_setup,
        f"{servo_rel} setup() must not attach GPIO 5 gate servo during boot",
        failures,
    )
    require(
        "rStop()" not in servo_setup,
        f"{servo_rel} setup() must not command GPIO 0 r servo during boot",
        failures,
    )
    require(
        "clawOpen()" not in servo_setup,
        f"{servo_rel} setup() must not command claw servo during boot",
        failures,
    )
    require(
        "gateClose()" not in servo_setup,
        f"{servo_rel} setup() must not command gate servo during boot",
        failures,
    )
    require(
        "attachRServoIfNeeded" in servo_text,
        f"{servo_rel} must lazily attach the GPIO 0 r servo only when commanded",
        failures,
    )
    require(
        "attachClawServoIfNeeded" in servo_text,
        f"{servo_rel} must lazily attach the GPIO 4 claw servo only when commanded",
        failures,
    )
    require(
        "attachGateServoIfNeeded" in servo_text,
        f"{servo_rel} must lazily attach the GPIO 5 gate servo only when commanded",
        failures,
    )
    servo_all_stop = extract_function_body(servo_text, "allStop")
    require(servo_all_stop, f"{servo_rel} must define allStop()", failures)
    require(
        "detachAllServoOutputs()" in servo_all_stop,
        f"{servo_rel} allStop() must detach GPIO 0/4/5 PWM outputs",
        failures,
    )
    require(
        "detachAllServoOutputs" in servo_text and "servoR.detach()" in servo_text,
        f"{servo_rel} must detach GPIO 0 r servo output on stop",
        failures,
    )
    require(
        "servoClaw.detach()" in servo_text,
        f"{servo_rel} must detach GPIO 4 claw servo output on stop",
        failures,
    )
    require(
        "servoGate.detach()" in servo_text,
        f"{servo_rel} must detach GPIO 5 gate servo output on stop",
        failures,
    )
    require(
        "GATE_OPEN_SPEED" in servo_text and "GATE_CLOSE_SPEED" in servo_text,
        f"{servo_rel} must treat the gate servo as timed 360-degree motion",
        failures,
    )
    require("GATE_STOP" in servo_text, f"{servo_rel} must define a gate servo stop pulse", failures)
    require(
        "GATE_RUN_TIME_MS" in servo_text,
        f"{servo_rel} must auto-stop timed gate motion",
        failures,
    )
    require(
        "checkGateTimeout" in servo_text,
        f"{servo_rel} must check and stop timed gate motion in loop()",
        failures,
    )

    main_rel = "esp32控制/01_MainController/01_MainController.ino"
    main_text = read(main_rel)
    for token in ("LEGSTOP", "TZSTOP", "SERVOSTOP", "SPD:", "GATEOPEN", "GATECLOSE"):
        require(token in main_text, f"{main_rel} must support {token}", failures)
    require(
        "normalizeBleCommandText" in main_text,
        f"{main_rel} must normalize BLE commands before matching text aliases",
        failures,
    )
    require(
        'normalizedCmd == "GATEOPEN"' in main_text,
        f"{main_rel} must accept spaced/underscored Gate Open BLE aliases",
        failures,
    )
    require(
        'normalizedCmd == "GATECLOSE"' in main_text,
        f"{main_rel} must accept spaced/underscored Gate Close BLE aliases",
        failures,
    )
    require(
        "COMMAND_KEEPALIVE_INTERVAL_MS" in main_text,
        f"{main_rel} must send keepalive packets for sustained leg movement",
        failures,
    )
    require("case 'I':" in main_text, f"{main_rel} must accept uppercase I for Theta+", failures)
    require("case 'K':" in main_text, f"{main_rel} must accept uppercase K for Theta-", failures)
    require(
        "uint8_t turntableZ_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}" in main_text,
        f"{main_rel} must keep C3 #2 MAC as placeholder until measured",
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
    for token in ("LEGSTOP", "TZSTOP", "SERVOSTOP", "STOP"):
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
        for token in ("BLE Controller – Arduino ESP32", "LEGSTOP", "TZSTOP", "SERVOSTOP", "STOP"):
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

    require("原地旋轉轉向" in root_readme, "README.md must describe walking turn mode as spin turn", failures)
    require("差速轉向" not in pickup_readme, "pickup README must not describe the current leg control as differential turn", failures)
    require("Servo ×4" in wiring_text, f"{wiring_rel} must consistently document four servos", failures)
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
