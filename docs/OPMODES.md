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
