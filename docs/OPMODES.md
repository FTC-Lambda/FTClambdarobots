# Current OpModes

| Driver Station name | Type | Purpose |
| --- | --- | --- |
| `LambdaGoWild` | TeleOp | Standard mecanum driving with squared joystick response and 0.5 drive sensitivity. |
| `Motor Calibration` | TeleOp | Adjust temporary per-motor drive scales and report them to telemetry. |
| `Limelight Pipeline Setup` | TeleOp | Configure Limelight pipeline 0 for FTC AprilTags. |
| `Limelight Test` | TeleOp | Manual driving plus scan/turn/drive test toward the closest observed tag. |
| `Limelight Seek Tag` | TeleOp | Manual driving or PID tag-following for selectable IDs 20–24. |
| `Ball Vision Test` | TeleOp | Inspect filtered/grouped ball detections and manually drive while selecting a preferred color. |
| `Ball-Grouping-Aligning test` | TeleOp | Manually drive or use PID/slew-limited rotation to center on the selected ball group's mean angle. |
| `Tuning` | TeleOp | Pedro Pathing's selectable localization, motion, and path tests. |
| `AprilTag Seek` | Autonomous | Configure/check pipeline 0, spin for a tag, center, and approach to 12 in. |

`BasicAuto.java` is empty and does not register an OpMode. `Arm`, `Intake`, and
`Shooter` are placeholders and are not used by any registered OpMode.
