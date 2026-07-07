AI Debugging Guide — Common Failures & Checks

Short, actionable checks for common FTC failures. Keep telemetry and logs concise.

- **General checklist:**
  - Verify hardware config names match `RobotHardware.java`.
  - Confirm battery voltage and connection to Control Hub.
  - Ensure latest app build deployed to Robot Controller.

- **PID oscillation**
  - Symptoms: repeated overshoot, sustained oscillation in motor output.
  - Quick checks: large loop delta time, derivative term too high, integral windup.
  - Remediation: reduce D, add I windup limits, clamp output, log loop dt.

- **Bad strafing / drift**
  - Symptoms: robot drifts while commanding straight movement.
  - Quick checks: motor scales, encoder mismatches, wheel slipping, calibration values in `Constants.java`.
  - Remediation: run motor calibration OpMode, verify wheel encoder counts are readable.

- **Vision loss / poor detections**
  - Symptoms: no AprilTag / Limelight results, intermittent detections.
  - Quick checks: Limelight web UI pipeline, camera USB connection, exposure settings, lighting conditions.
  - Remediation: switch to a robust pipeline, add retries/timeouts, fallback behavior if tag lost.

- **Encoder issues**
  - Symptoms: odometry diverges, encoders read zero or noisy values.
  - Quick checks: confirm `DcMotor` mode (RUN_USING_ENCODER vs RUN_WITHOUT_ENCODER), wiring, connectors.
  - Remediation: power cycle hub, re-seat cables, log raw counts per-loop.

- **IMU / heading failures**
  - Symptoms: unstable heading, sudden jumps.
  - Quick checks: sensor initialization time, mounting orientation, calibration state.
  - Remediation: re-run IMU calibration, apply mounting transform, filter noisy readings.

- **Localization failures**
  - Symptoms: pose drifts over time, resets to wrong values.
  - Quick checks: verify odometry implementation, unit conversions, and pose reset calls.
  - Remediation: add sanity checks (max delta per cycle), allow re-localization from vision.

- **AprilTag-specific debugging**
  - Symptoms: wrong tag ID, incorrect range/bearing.
  - Quick checks: camera calibration, Limelight AprilTag pipeline parameters, tag stickers orientation.
  - Remediation: verify tag size/pose, tune detection thresholds, log raw fiducial results.

- **Telemetry checklist for failed runs**
  - Include: battery voltage, loop dt, motor powers, encoder counts, IMU heading, latest vision result, current OpMode phase.
  - Log short, structured JSON lines for agent parsing when possible.

TODOs:
- Add device-specific troubleshooting steps after reviewing `RobotHardware.java` and `subsystems` code.
