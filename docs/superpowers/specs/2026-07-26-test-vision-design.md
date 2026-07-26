# Test-Vision Design

## Goal

Replace the existing Limelight test-only drive/AprilTag sequence with a TeleOp results viewer named `Test-Vision` that continuously displays the currently selected Limelight pipeline's output on Driver Station telemetry.

## Architecture

`LimelightTest` remains a `LinearOpMode` and initializes hardware through the existing `RobotHardware` class. It configures polling and starts the existing `robot.limelight` instance once before the match loop; no second Limelight wrapper or hardware lookup is introduced.

Every active-loop iteration safely obtains `LLStatus` and `LLResult`. Telemetry renders three sections: connection/pipeline status, frame validity and latency, and the detected targets. Detector results are the selected detector pipeline's targets. Each target reports its class label, confidence, tx, ty, and target area. Result-level robot pose and detector camera-space pose are printed only when available.

## Error Handling

All reads that can fail or return unavailable data are guarded. A disconnected Limelight, null status/result, invalid result, unavailable result collections, or runtime communication exception produces a readable telemetry message and allows the OpMode to continue. If a valid frame has no detector targets, telemetry explicitly says `No targets detected.`

## Scope

- Keep this as a `@TeleOp` and set its Driver Station name to `Test-Vision`.
- Reuse the project’s existing Limelight initialization in `RobotHardware`.
- Poll with `getLatestResult()` every control loop.
- Use only methods confirmed by the project’s installed FTC/Limelight libraries.
- Remove only vision files that are confirmed unused by all production code and tests; preserve files with active references.
- Do not alter unrelated OpModes, robot hardware, or production vision logic.

## Verification

Compile the TeamCode Android module after the change. The test OpMode’s behavior is hardware-facing and cannot be unit-tested without FTC hardware; compilation validates all Limelight method calls against the installed SDK.
