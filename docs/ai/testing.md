# Testing

## Source references
- `/home/runner/work/FTClambdarobots/FTClambdarobots/README.md`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/MotorCalibrationOpMode.java`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/LimelightTest.java`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/docs/superpowers/plans/2026-06-24-apriltag-seek-auto.md`

## Robot test procedures
- Build/deploy from Android Studio (or existing Gradle workflow when network allows dependency resolution).
- Validate hardware init before motion tests (motor + limelight mapping names).
- Test manual drive before autonomous.

## On-block tests
- Verify each drive motor direction and response at low power.
- Run `Motor Calibration` OpMode to confirm scale adjustment controls.
- Verify Limelight connection and fiducial pipeline status telemetry.

## Carpet tests
- Drive straight/strafe/rotate and observe drift.
- Re-run motor calibration adjustments after carpet runs.
- Verify stop behavior when controls return to neutral.

## Autonomous validation
- For `AprilTagSeekAuto`, validate:
  - `SPIN`: finds a tag or cleanly times out.
  - `CENTER`: reduces bearing error and transitions.
  - `APPROACH`: moves toward tag and stops near desired distance.
- Use telemetry to confirm phase transitions and target lock state.

## TeleOp validation
- `MainTeleOp`: joystick mapping and live telemetry fields.
- `LimelightTest`: manual/scan/turn/drive mode transitions.
- `LimelightSeekTag`: manual/follow transitions and deadband tuning controls.

## Safety checklist
- Wheels off-ground for first motion after code changes.
- Confirm emergency stop input (`B` in Limelight test OpModes) before full-floor runs.
- Keep clear field perimeter during autonomous tests.
- TODO: add team-specific E-stop and spotter protocol.

## Competition checklist
- Confirm hardware configuration names match deployed robot configuration.
- Confirm Limelight pipeline 0 is fiducial before match.
- Confirm autonomous OpMode selection and start pose setup.
- Confirm driver controls for selected TeleOp are reviewed pre-match.
- TODO: add match-day battery and preflight checklist used by the team.
