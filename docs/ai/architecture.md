# AI architecture notes

This file summarizes the codebase structure for developer agents and human maintainers. Treat source files as authoritative when this document disagrees with code.

## Key packages

| Path | Purpose |
| --- | --- |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto` | autonomous OpModes |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop` | TeleOp and setup OpModes |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware` | `RobotHardware.java` hardware map access |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems` | drivetrain plus placeholder mechanism subsystem classes |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util` | constants, PID, rate limit, vision helpers |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing` | Pedro Pathing constants and tuning OpMode |

## Confirmed runtime structure

- `RobotHardware` retrieves four drive motors and the Limelight from the FTC hardware map.
- `Drivetrain` owns mecanum drive power calculation for regular TeleOp and autonomous code.
- `MainTeleOp` runs standard robot-centric mecanum driving with squared joystick response and configured drive sensitivity.
- `MotorCalibrationOpMode` tunes temporary per-motor drivetrain scale factors and reports values through telemetry.
- Limelight TeleOps and `AprilTagSeekAuto` use pipeline `0` for AprilTag work.
- Pedro Pathing is configured separately through `pedroPathing/Constants.java` and the `Tuning` TeleOp.

## Placeholder code

`Arm`, `Intake`, and `Shooter` exist as subsystem placeholders but do not retrieve hardware or control real devices yet. Do not invent hardware map names for them.

`BasicAuto.java` is empty and does not register an OpMode.

## Agent rules

Before changing robot behavior:

1. Read `RobotHardware.java` before touching hardware names.
2. Read the relevant OpMode and subsystem end to end.
3. Reuse constants and helpers in `util` before adding new control code.
4. Keep autonomous loops stop-aware, timeout-aware, and able to stop motors.
5. Treat physical behavior as unconfirmed until humans test the robot.
