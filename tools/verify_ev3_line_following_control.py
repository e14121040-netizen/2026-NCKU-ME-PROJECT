#!/usr/bin/env python3
"""Verify EV3 line-following control keeps turn authority when off line."""

from __future__ import annotations

import ast
import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PROGRAM_PATH = next(ROOT.glob("*/ev3_program/transport_robot.py"))
EV3_ROOT = PROGRAM_PATH.parents[1]
TYPINGS_PATH = EV3_ROOT / "typings"


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def has_main_guard(tree: ast.Module) -> bool:
    for node in tree.body:
        if not isinstance(node, ast.If):
            continue
        test = node.test
        if (
            isinstance(test, ast.Compare)
            and isinstance(test.left, ast.Name)
            and test.left.id == "__name__"
            and len(test.ops) == 1
            and isinstance(test.ops[0], ast.Eq)
            and len(test.comparators) == 1
            and isinstance(test.comparators[0], ast.Constant)
            and test.comparators[0].value == "__main__"
        ):
            return True
    return False


def has_function(tree: ast.Module, name: str) -> bool:
    return any(isinstance(node, ast.FunctionDef) and node.name == name for node in tree.body)


def load_transport_robot():
    sys.path.insert(0, str(TYPINGS_PATH))
    spec = importlib.util.spec_from_file_location("transport_robot_under_test", PROGRAM_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("Unable to load transport_robot.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeSensor:
    def __init__(self, reflection: int) -> None:
        self._reflection = reflection

    def reflection(self) -> int:
        return self._reflection


class FakeRobot:
    def __init__(self) -> None:
        self.commands: list[tuple[int, int]] = []

    def drive(self, speed: int, turn_rate: int) -> None:
        self.commands.append((speed, turn_rate))


def main() -> int:
    failures: list[str] = []

    source = PROGRAM_PATH.read_text()
    tree = ast.parse(source)

    require(
        has_function(tree, "compute_line_drive_command"),
        "transport_robot.py should expose compute_line_drive_command() for line-following control.",
        failures,
    )
    require(
        has_main_guard(tree),
        "transport_robot.py should guard main() with if __name__ == '__main__' so control helpers can be tested.",
        failures,
    )

    if failures:
        for failure in failures:
            print("FAIL:", failure)
        return 1

    robot_module = load_transport_robot()

    large_white_error = -35
    white_raw = robot_module.PIDController(
        robot_module.KP, robot_module.KI, robot_module.KD
    ).compute(large_white_error) * robot_module.PID_OUTPUT_SCALE
    white_speed, white_turn = robot_module.compute_line_drive_command(
        large_white_error, white_raw
    )
    require(
        white_speed == 0,
        "large white-side error should stop forward motion while searching for the line.",
        failures,
    )
    require(
        white_turn >= robot_module.LINE_SEARCH_MIN_TURN_RATE,
        "large white-side error should keep enough right turn rate to recover.",
        failures,
    )

    large_black_error = 35
    black_raw = robot_module.PIDController(
        robot_module.KP, robot_module.KI, robot_module.KD
    ).compute(large_black_error) * robot_module.PID_OUTPUT_SCALE
    black_speed, black_turn = robot_module.compute_line_drive_command(
        large_black_error, black_raw
    )
    require(
        black_speed == 0,
        "large black-side error should stop forward motion while searching for the line.",
        failures,
    )
    require(
        black_turn <= -robot_module.LINE_SEARCH_MIN_TURN_RATE,
        "large black-side error should keep enough left turn rate to recover.",
        failures,
    )

    mild_error = -5
    mild_raw = robot_module.PIDController(
        robot_module.KP, robot_module.KI, robot_module.KD
    ).compute(mild_error) * robot_module.PID_OUTPUT_SCALE
    mild_speed, mild_turn = robot_module.compute_line_drive_command(mild_error, mild_raw)
    require(mild_speed > 0, "small line error should keep driving forward.", failures)
    require(
        abs(mild_turn) < robot_module.LINE_SEARCH_MIN_TURN_RATE,
        "small line error should not use search turn rate.",
        failures,
    )

    robot_module.sensor_line = FakeSensor(robot_module.LINE_THRESHOLD - large_white_error)
    robot_module.robot = FakeRobot()
    robot_module.pid = robot_module.PIDController(
        robot_module.KP, robot_module.KI, robot_module.KD
    )
    robot_module.check_stop_color = lambda: None

    stop_color = robot_module.follow_line()

    require(stop_color is None, "follow_line() should keep following when no stop color is present.", failures)
    require(
        robot_module.robot.commands == [(white_speed, white_turn)],
        "follow_line() should drive using compute_line_drive_command().",
        failures,
    )

    if failures:
        for failure in failures:
            print("FAIL:", failure)
        return 1

    print("EV3 line-following control checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
