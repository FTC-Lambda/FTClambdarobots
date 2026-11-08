# Current OpModes

| Driver Station name | Type | Purpose |
| --- | --- | --- |
| `LambdaGoWild` | TeleOp | Standard mecanum driving with squared joystick response and 0.5 drive sensitivity. |
| `Motor Calibration` | TeleOp | Adjust temporary per-motor drive scales and report them to telemetry. |
| `Limelight Pipeline Setup` | TeleOp | Configure Limelight pipeline 0 for FTC AprilTags. |
| `Limelight Test` | TeleOp | Manual driving plus scan/turn/drive test toward the closest observed tag. |
| `Limelight Seek Tag` | TeleOp | Manual driving or PID tag-following for selectable IDs 20–24. |
| `Ball Vision Test` | TeleOp | Inspect filtered/grouped ball detections and manually drive while selecting a preferred color. |
| `Ball-Grouping-Aligning test` | TeleOp | Manually drive or use the shared PID/slew-limited controller to center on the selected ball group's filtered control bearing. |
| `Ball-Group Alignment Tuner` | TeleOp | Tune ball targeting and turn-alignment values live while viewing complete target/controller diagnostics. |
| `Tuning` | TeleOp | Pedro Pathing's selectable localization, motion, and path tests. |
| `Hive State Test` | TeleOp | Pick alliance in INIT (X=BLUE, B=RED); manually drive while telemetry shows which CELL of your HIVE faces up, per-CELL tag heights, and rumbles on a confirmed change. |
| `Hive Localization Test` | TeleOp | Field pose (x, y, heading, tile) from both HIVEs' tags. Tags-only fit from 2+ tags; with a Pinpoint, A aligns its gyro to the field and then any single tag gives x/y. Shows fit quality, height error and jitter. See [HIVE localization](game/HIVE_LOCALIZATION.md). |
| `Hive Tag Telemetry` | TeleOp | Raw Limelight readout for HIVE tags 30–45 only, no state logic: per tag `tx/ty` (from the lens center), area, 3D range, height above the tiles, plus Limelight FPS, temperature and latency. Y toggles camera-space x/y/z and yaw. |
| `Hive Height Calibration` | TeleOp | Record RAISED/LOWERED CELL tag heights (A/Y, X clears) to get a suggested threshold and hysteresis; D-pad nudges camera pitch/height for a runtime sanity check. |
| `AprilTag Seek` | Autonomous | Configure/check pipeline 0, spin for a tag, center, and approach to 12 in. |

`BasicAuto.java` is empty and does not register an OpMode. `Arm`, `Intake`, and
`Shooter` are placeholders and are not used by any registered OpMode.

## Ball group alignment controls

Both ball alignment OpModes use `A` to enter alignment, `B` to return to manual
driving, `X` to request green balls, `Y` to request purple balls, and either
bumper to accept any ball color.

In `Ball-Group Alignment Tuner`, D-pad up/down selects a parameter and D-pad
left/right changes it. Hold the left-stick button while pressing left/right for
the coarse increment; otherwise the fine increment is used. Every button change
is edge-triggered, so a held button does not repeat.

Tuner values are runtime-only and reset to their defaults when the OpMode ends.
After validating a combination on the robot, copy those values into the
corresponding alignment and targeting defaults before using them in competition.

## HIVE state detection OpModes

Both `Hive State Test` and `Hive Height Calibration` select the alliance during INIT
(`X` = BLUE, `B` = RED, default RED), start the Limelight on pipeline 0 only after START,
and drive manually with the sticks. Nothing moves during INIT (G403). Both run with only
the Limelight configured; the drivetrain is optional.

`Hive State Test` also scores accuracy against the real HIVE. Enter the truth with D-pad
up (FAR UP), D-pad down (AUDIENCE UP) or D-pad left (stop scoring), and press A to start a
fresh tally at each test spot. Names and the test protocol are in
[HIVE naming and testing](game/HIVE_NAMING.md).

`Hive Height Calibration` records the median height of the visible own-alliance CELL tags:
`A` = RAISED sample, `Y` = LOWERED sample, `X` = clear. Point the camera at only one of
your CELLs while recording; if both are in view it refuses the sample. It suggests
`raisedThresholdIn = (maxLowered + minRaised) / 2` and
`hysteresisIn = max(0.5, (minRaised - maxLowered) / 4)`, and warns if the ranges overlap.
D-pad up/down changes camera pitch by 0.5 deg and left/right changes camera height by
0.25 in so the shown tag height can be checked against a tape measure. All values are
runtime-only: copy the camera mount into `Constants.LIMELIGHT_HEIGHT_IN` /
`LIMELIGHT_PITCH_DEG` and the threshold into the `HiveDetectionConfig` defaults.
