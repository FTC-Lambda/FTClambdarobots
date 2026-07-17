# AI testing procedures

These are short robot-side checks for validating drivetrain, vision, autonomous, and safety behavior. They do not replace physical inspection.

## On-block quick tests

1. Confirm battery voltage is healthy before motion testing.
2. Initialize the target OpMode and verify no missing hardware-map errors appear.
3. Run each drive direction at low power and confirm the expected wheel response.
4. Watch telemetry for all four motor powers and motor scale values.

## Motor calibration

Use `Motor Calibration` after drivetrain hardware changes.

Current default scales:

| Motor | Scale |
| --- | --- |
| `top_left` | `1.00` |
| `back_left` | `1.40` |
| `top_right` | `1.00` |
| `back_right` | `1.40` |

Tune one motor scale at a time and keep output power low until direction and naming are confirmed.

## Carpet drive tests

1. Test pure forward, pure strafe, and pure turn separately.
2. Mark a known start line and target line.
3. Record drift, rotation, battery voltage, and final robot heading.
4. Repeat after calibration changes before raising speed.

Accept rough drift during early tuning. Do not mask major mechanical or wiring problems with software constants.

## AprilTag autonomous validation

Setup:

- Place an AprilTag about `1 m` in front of the robot.
- Confirm Limelight pipeline `0` is an AprilTag pipeline.
- Confirm the configured tag size matches the printed target.

Run `AprilTag Seek` and verify:

1. Search does not drive forever without timeout.
2. The robot centers on the tag instead of turning away.
3. Approach stops near the configured desired distance.
4. Motors stop when the OpMode finishes, times out, or is stopped.

Collect telemetry for tag ID, yaw or bearing, range, state, elapsed state time, and motor powers.

## TeleOp validation

Run `LambdaGoWild` and confirm:

- Forward stick command drives forward.
- Strafe command moves laterally.
- Turn command rotates the expected direction.
- Releasing sticks stops the drivetrain.
- Driver Station stop immediately stops motion.

## Safety checklist

- Emergency stop tested and reachable.
- Robot starts on blocks for first motion test after code or wiring changes.
- Battery is secured.
- Wiring cannot enter wheels or belts.
- Autonomous has a human ready to stop the run.
- Robot behavior is not considered fixed until physically tested.
