# Architecture

## Source references
- `/home/runner/work/FTClambdarobots/FTClambdarobots/README.md`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/TeamCode/src/main/java/org/firstinspires/ftc/teamcode`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/docs/superpowers/specs/2026-06-24-apriltag-seek-auto-design.md`

## High-level robot architecture
- FTC SDK Android project with team logic in `TeamCode`.
- OpModes orchestrate runtime behavior:
  - TeleOp: `MainTeleOp`, `MotorCalibrationOpMode`, Limelight test/setup OpModes.
  - Autonomous: `AprilTagSeekAuto` (`BasicAuto` is currently empty).
- `RobotHardware` owns hardware-device lookup and exposes devices to subsystems.

## Package structure
- `auto/` - autonomous OpModes.
- `teleop/` - driver and setup/test OpModes.
- `hardware/` - hardware map + Limelight pipeline helpers.
- `subsystems/` - `Drivetrain`, `Arm`, `Intake`, `Shooter` (last three are placeholders).
- `util/` - shared constants and vision deadband logic.

## Subsystems
- Implemented:
  - `Drivetrain` (mecanum drive math, calibration scales, raw + joystick-shaped drive APIs).
- Declared placeholders:
  - `Arm`, `Intake`, `Shooter` (TODO: document when implemented).

## Control flow
- TeleOp control flow:
  - `MainTeleOp`: read gamepad -> `Drivetrain.drive(...)` -> telemetry update loop.
  - Limelight TeleOp modes: switch between manual and vision-driven behavior in loop state machines.
- Autonomous control flow:
  - `AprilTagSeekAuto`: `SPIN -> CENTER -> APPROACH -> DONE` loop phases.

## Command flow
- No FTC command-based framework is currently defined.
- Current command flow is direct method invocation from OpModes to subsystems/hardware.
- TODO: add command scheduler/command mapping documentation if adopted.

## Vision pipeline
- Camera device: Limelight 3A via `RobotHardware.limelight`.
- Expected AprilTag pipeline index: `0`.
- Pipeline setup path:
  - Use `LimelightPipelineSetup` to enforce fiducial settings when needed.
  - `AprilTagSeekAuto` also calls pipeline self-heal (`LimelightPipelines.ensureFiducial(...)`).

## Autonomous structure
- Implemented autonomous:
  - `AprilTagSeekAuto` with three sequential phases and timeout guards.
- TODO:
  - Document additional autonomous routines when `BasicAuto` or new autos are implemented.
