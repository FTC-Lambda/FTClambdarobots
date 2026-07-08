AI Hardware Notes

> **This document reflects the current codebase, not necessarily the physical robot.**
> If hardware changes on the robot before the code is updated, `RobotHardware.java` remains the software source of truth until synchronized.

Authoritative source: `TeamCode/src/main/java/org/firstinspires.ftc.teamcode/hardware/RobotHardware.java`.

- **Hardware map (from `RobotHardware.java`):**
  - Motors (DcMotor):
    - `topLeft` — hardware name: `top_left`
    - `topRight` — hardware name: `top_right`
    - `backLeft` — hardware name: `back_left`
    - `backRight` — hardware name: `back_right`
  - Camera:
    - Limelight 3A: `limelight` (type: `Limelight3A`)

- **Notes / unknowns (do not invent):**
  - Encoder presence, motor directions, and per-motor scaling are declared elsewhere; see below.
  - No servos or IMU are declared in `RobotHardware.java`; list any additional devices by inspecting the rest of the `hardware` package.
  - USB webcam calibrations file exists at `TeamCode/src/main/res/xml/teamwebcamcalibrations.xml` (contains commented example entries; no active camera calibrations found).

- **Drive & motor calibration (from `Constants.java` and `Drivetrain.java`):**
  - `DRIVE_SPEED`: 0.5 (applied after input squaring in `Drivetrain.drive()`)
  - Motor scale bounds: `MOTOR_SCALE_MIN` = 0.0, `MOTOR_SCALE_MAX` = 2.0, step = `MOTOR_SCALE_STEP` = 0.05
  - Default per-motor scales (field-tuned):
    - `TOP_LEFT_SCALE` = 1.00 (applied to `top_left`)
    - `BACK_LEFT_SCALE` = 1.40 (applied to `back_left`)
    - `TOP_RIGHT_SCALE` = 1.00 (applied to `top_right`)
    - `BACK_RIGHT_SCALE` = 1.40 (applied to `back_right`)
  - `Drivetrain` applies these scales after normalization and clamps power to [-1, 1]. `driveRaw(...)` is available for raw-power autonomous control (also applies scales).

- **Encoders & modes:**
  - I found no explicit `setMode(...)` or calls to `getCurrentPosition()` in the codebase, so motors are not configured for encoder-based closed-loop control in `RobotHardware`/`Drivetrain`.
  - TODO: if encoders are required for an OpMode, update `RobotHardware.init()` or OpMode init to set `RUN_USING_ENCODER` and/or read positions.

- **Servos:**
  - No `Servo` or `CRServo` usages were found in `TeamCode/` — none documented here.

- **IMU / sensors:**
  - No IMU (`BNO055IMU` or similar) or other sensors are declared in `RobotHardware.java` or elsewhere in `TeamCode/`.
  - TODO: add IMU or other sensor entries once present in code or hardware config.

- **Cameras / vision:**
  - `limelight` (Limelight 3A) — used by vision OpModes such as `LimelightSeekTag` and `LimelightTest` (see `TeamCode/src/main/java/.../teleop`).
  - Pipeline selection and exposure are controlled in the Limelight web UI; ensure pipeline 0 is configured for AprilTag if using AprilTag OpModes.

- **Control Hub / Rev Hub:**
  - Hardware platform (Control Hub vs Expansion Hub) is not declared in code; this is configured on-device. TODO: document hub types and firmware versions from your robot inventory.

- **Wiring notes:**
  - TODO: add wiring diagram references or photos.

- **Cameras:**
  - `limelight` (Limelight 3A) — pipeline configuration is external (Limelight web UI). Ensure pipeline 0 is set to AprilTag if using AprilTag OpModes.

- **Guidance for updates:**
  - Update `RobotHardware.java` first when adding/removing devices, then mirror concise entries here.
  - Keep entries factual; if a detail is unknown, leave a TODO referencing the source file.
