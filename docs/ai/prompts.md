# Prompts

## Reusable prompts for Codex
- "Use only facts from `/home/runner/work/FTClambdarobots/FTClambdarobots/TeamCode` and `/home/runner/work/FTClambdarobots/FTClambdarobots/docs`. If unknown, output TODO instead of guessing."
- "Keep output concise and bullet-based. Do not invent hardware names or subsystem names."

## Research prompts
- "Summarize implemented robot architecture from `/home/runner/work/FTClambdarobots/FTClambdarobots/TeamCode/src/main/java/org/firstinspires/ftc/teamcode` with file references."
- "List current hardware map names from `RobotHardware.java` and mark missing categories with TODO."

## Implementation prompts
- "Create minimal documentation updates under `/home/runner/work/FTClambdarobots/FTClambdarobots/docs/ai` by linking to existing docs instead of duplicating text."
- "Update only the requested markdown files; keep section headings stable."

## Code review prompts
- "Review only changed files and flag factual inaccuracies or invented robot details."
- "Check that all hardware/subsystem names in docs exactly match the Java source names."

## Debug prompts
- "From Limelight OpModes, generate a quick triage checklist for 'no AprilTag detected' with concrete telemetry checks."
- "Given drivetrain scaling constants, suggest a minimal calibration workflow without changing robot logic."

## Robot testing prompts
- "Draft an on-block test checklist using only existing OpModes and telemetry fields."
- "Draft autonomous validation steps for `AprilTagSeekAuto` phase transitions (`SPIN`, `CENTER`, `APPROACH`)."
