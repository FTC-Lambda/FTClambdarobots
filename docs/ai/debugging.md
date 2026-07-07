# Debugging

## Source references
- `/home/runner/work/FTClambdarobots/FTClambdarobots/README.md`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/LimelightTest.java`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/LimelightSeekTag.java`
- `/home/runner/work/FTClambdarobots/FTClambdarobots/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/LimelightPipelineSetup.java`

## Common FTC failures
- Hardware-name mismatch -> `hardwareMap.get(...)` failure on init.
- Limelight not connected/pipeline not fiducial -> no AprilTag detections.
- Motor calibration drift -> robot pulls left/right under straight drive.

## PID oscillation
- Current code primarily uses proportional gains (`TURN_GAIN`, `DRIVE_GAIN`) and deadbands.
- If oscillation appears in tag-following or centering:
  - reduce `TURN_GAIN` and/or `DRIVE_GAIN`
  - increase deadband thresholds
  - verify minimum turn feedforward is not too aggressive
- TODO: no closed-loop PID controller docs currently exist.

## Bad strafing
- Use `Motor Calibration` OpMode procedure from `README.md`.
- Re-check `TOP_LEFT/BACK_LEFT/TOP_RIGHT/BACK_RIGHT` scale values.
- Confirm floor conditions match tuning environment.

## Vision loss
- Confirm `robot.limelight.isConnected()` telemetry.
- Confirm pipeline type shows fiducial (`pipe_fiducial`).
- Run `Limelight Pipeline Setup` if pipeline was reset or incorrect.

## Encoder issues
- TODO: encoder troubleshooting flow is not currently documented.

## IMU issues
- TODO: IMU configuration/troubleshooting is not currently documented.

## Localization failures
- TODO: localization stack is not currently documented in this repository.

## AprilTag debugging
- Confirm pipeline index `0` and fiducial family settings.
- Confirm tag size (mm) used in pipeline config matches printed tag.
- Use `Limelight Test` telemetry to verify:
  - tag visibility
  - bearing (deg)
  - distance (in)
- If distance remains zero/invalid, verify 3D fiducial solve configuration.

## Telemetry checklist
- During debug, verify these fields are updating:
  - Limelight connection status
  - Pipeline index/type
  - Target tag ID / visible count
  - Bearing and distance
  - Mode/action state
  - Drive and turn command outputs
