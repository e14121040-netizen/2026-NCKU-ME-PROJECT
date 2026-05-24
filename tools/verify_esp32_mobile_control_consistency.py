from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(rel_path: str) -> str:
    return (ROOT / rel_path).read_text(encoding="utf-8")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


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

    main_rel = "esp32控制/01_MainController/01_MainController.ino"
    main_text = read(main_rel)
    for token in ("LEGSTOP", "TZSTOP", "SERVOSTOP", "SPD:", "GATEOPEN", "GATECLOSE"):
        require(token in main_text, f"{main_rel} must support {token}", failures)

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

    if failures:
        print("ESP32 mobile control consistency check FAILED:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("ESP32 mobile control consistency check PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
