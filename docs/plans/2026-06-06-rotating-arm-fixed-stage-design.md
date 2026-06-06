# Rotating Arm / Fixed Stage Controller Design

## Goal

Avoid wire twisting by matching each ESP32-C3 controller to the mechanism that physically moves with it.

## Mechanical Boundary

- C3 #2 is mounted on the rotating upper arm.
- C3 #3 remains on the fixed lower stage near the temporary box.
- The upper arm only rotates within a limited range such as +/-90 deg or +/-180 deg, so no slip ring is required.
- Wires crossing the rotation joint should pass near the rotation center and use a slack loop with strain relief.

## Controller Split

### C3 #2 RotatingArm

C3 #2 controls mechanisms that rotate with the upper arm:

- Retract / Extend continuous servo for the rack.
- Theta +/- standard servo.
- Claw Open / Close standard servo, because it is part of the upper gripper.
- Z +/- DC gear motor through BTS7960.

### C3 #3 FixedStage

C3 #3 controls mechanisms fixed to the lower stage:

- Spin right / left DC gear motor through BTS7960.
- Gate Open / Close servo for the temporary box.

## Command Routing

- `EXTEND`, `RETRACT`, `RSTOP`, `THETA+`, `THETA-`, `OPEN`, `CLOSE`, `UP`, `DOWN`, `HOME`, `ARMSTOP`, `TZSTOP`, and `SERVOSTOP` route to C3 #2.
- `TL`, `TR`, `GATEOPEN`, `GATECLOSE`, and `FIXEDSTOP` route to C3 #3.
- `HOME` sends an arm home command to C3 #2 and a gate close command to C3 #3.
- `STOP` still queues stop commands for all child controllers.

## Compatibility

Existing app command strings are preserved where practical:

- `TL` / `TR` still control the rotating alignment motor, but the motor is now on fixed C3 #3.
- `TZSTOP` remains accepted as a C3 #2 stop because C3 #2 still owns Z.
- `SERVOSTOP` remains accepted as a C3 #2 stop because the upper arm servos moved there.
- `FIXEDSTOP` is added for stopping C3 #3 fixed-stage motion.

## Verification

The consistency script will check that:

- Protocol names expose `RotatingArm` and `FixedStage` messages.
- C3 #2 includes both `ESP32Servo` and BTS7960 Z control.
- C3 #3 includes fixed spin motor control and timed gate servo control.
- Main controller routes arm, Z, fixed spin, and gate commands to the correct peer.
- Wiring and ESP32 README documentation describe the new physical split.
