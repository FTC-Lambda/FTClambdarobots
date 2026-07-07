AI Testing — Robot Test Procedures

Short test procedures for validating drive, autonomous, and safety checks.

- **On-block quick tests**
  - Battery & basics: confirm battery > 12.0V, robot powers on, no ERR on driver station.
  - Motor smoke test: run each motor at low power for 2s and confirm expected direction.

- **Carpet strafing test**
  - Procedure: mark start and target 2m apart on carpet. Command robot to strafe, measure offset.
  - Tolerances: ±10 cm for initial tuning; tighten after calibration.
  - Telemetry: encoder counts (start/end), IMU heading before/after, motor scales.

- **Autonomous validation (AprilTag SEEK)**
  - Setup: place AprilTag at ~1m, ensure Limelight pipeline 0 is AprilTag.
  - Steps: run OpMode, verify SPIN finds tag, CENTER aligns within bearing threshold, APPROACH stops at DESIRED_DISTANCE.
  - Collect: loop dt, latest fiducial result, range estimate, motor powers, and final pose.

- **TeleOp validation**
  - Confirm joystick axes map, test all subsystems (intake, shooter, arm) with safe power limits.

- **Safety checklist before competition**
  - Emergency stop tested and reachable.
  - Battery fully charged and secured.
  - No loose wiring or hardware interferences.
  - Team aware of run/stop procedures.

TODOs:
- Add expected encoder counts per wheel for known distances once wheel circumference and gear ratios are documented in `Constants.java`.
