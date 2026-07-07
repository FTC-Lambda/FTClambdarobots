# Hardware

## Source references
- `/home/runner/work/FTClambdarobots/FTClambdarobots/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/RobotHardware.java`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/README.md`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/docs/superpowers/specs/2026-06-24-apriltag-seek-auto-design.md`

## Hardware map
- Motors (`DcMotor`):
  - `top_left`
  - `back_left`
  - `top_right`
  - `back_right`
- Camera:
  - `limelight` (`Limelight3A`)

## Motors
- Drive motors are used by `Drivetrain` mecanum equations.
- Per-motor scale constants:
  - `TOP_LEFT_SCALE`
  - `BACK_LEFT_SCALE`
  - `TOP_RIGHT_SCALE`
  - `BACK_RIGHT_SCALE`

## Servos
- TODO: no servo mapping is currently defined in `RobotHardware`.

## Sensors
- TODO: no non-camera sensors are currently mapped in `RobotHardware`.

## Cameras
- Limelight 3A is required for AprilTag workflows.
- Expected pipeline for AprilTag operation: pipeline `0` as fiducial.

## Control Hub configuration
- Device-name strings must match:
  - `back_left`, `back_right`, `top_left`, `top_right`, `limelight`.
- Limelight is documented as connected to Control Hub USB 3.0 in existing design spec.

## Wiring notes
- Keep Limelight connected and reachable before running AprilTag OpModes.
- TODO: add motor-port mapping and power-distribution notes.
- TODO: add hub/expansion-hub topology notes if used.
