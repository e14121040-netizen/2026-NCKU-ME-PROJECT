# Gate Position Hold Design

## Context

`esp32控制/04_FixedStageController/04_FixedStageController.ino` currently treats the gate as a timed motion: `GATE_OPEN_SPEED = 180`, `GATE_CLOSE_SPEED = 0`, and `GATE_RUN_TIME_MS = 500`. When the timeout completes, `gateStop()` detaches the servo PWM. This makes the gate travel depend on elapsed time and leaves the closed gate free to move if a part pushes on it.

The confirmed hardware is a 180-degree positional servo, so the gate should be controlled by target angles rather than timed continuous rotation.

## Requirements

- Make the gate open/close travel slightly smaller than the current full 0-to-180 command span.
- Keep the gate fixed in the closed position so stored parts cannot push it open.
- Preserve `FIXEDSTOP` / serial `0` as the explicit command that releases the gate PWM.
- Keep boot behavior unchanged: do not attach or command the gate servo during setup.
- Keep the change scoped to fixed-stage gate control, verification, and related docs.

## Design

Replace the timed gate-speed constants with positional constants:

- `GATE_OPEN_ANGLE = 160`
- `GATE_CLOSE_ANGLE = 20`

These values reduce travel by 20 degrees at each end compared with the previous 0-to-180 span and avoid driving the servo into likely mechanical end stops. They are conservative starting values for field calibration.

`gateOpen()` will attach the servo and write `GATE_OPEN_ANGLE`. It may release PWM after a short hold window because the open gate does not need anti-push holding.

`gateClose()` will attach the servo and write `GATE_CLOSE_ANGLE`. After the motion completes, the servo stays attached and keeps receiving the close angle, which provides holding torque against parts pushing on the gate.

`gateStop()` remains the explicit release path. `allStop()` continues to call `gateStop()`, so `FIXEDSTOP` and serial `0` still stop fixed-stage motion and disable the gate PWM.

## Verification

Update `tools/verify_esp32_mobile_control_consistency.py` to require angle-based gate constants and to reject the old timed-speed gate behavior. Add checks that `gateClose()` holds the servo at the close angle and that the gate timeout path does not call `gateStop()` after a close command.

Run the Python verifier and compile `esp32控制/04_FixedStageController/04_FixedStageController.ino` with `arduino-cli compile`.
