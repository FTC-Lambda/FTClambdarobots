# Lambda FTC — robot code

FTC team Lambda (Ho Chi Minh City), **2026–2027 season: BIOBUZZ presented by
RTX**. Java, FTC SDK, Pedro Pathing, Limelight 3A.

## Game context: read before changing game logic

- `docs/game/BIOBUZZ.md`: game digest with rule numbers (timing, POLLEN and
  NECTAR, HIVE/CELL/FLOWER, AprilTags 30–45 at 82.55 mm, scoring, RPs, rules
  that constrain software).
- `docs/game/CODE_IMPACT.md`: where this repo still assumes DECODE (2025–26),
  in priority order.
- `docs/game/HIVE_NAMING.md`: HIVE CELL/state naming (FAR/AUDIENCE, `R-F`…), the
  driver-relative LEFT/RIGHT mapping, and the real-HIVE test protocol.
  Code: `vision/hive/`.
- `docs/game/HIVE_LOCALIZATION.md`: field frame (origin A1, +x→F, +y→far wall), the HIVE
  tag-position model and its assumptions, and how `HiveLocalizer` fits a pose.
- `docs/game/reference/`: official PDFs, git-ignored. Run `./fetch.sh` there to
  download them. The Competition Manual is the final authority.

Several parts of the code (green/purple `BallColor`, tag IDs 20–24, 165.1 mm tag
size) still describe DECODE. Don't copy those values into new code.

## Rules that bite software

- R704.D: FTC Dashboard and FTControl Panels streaming is **prohibited at
  events**. Panels (`com.bylazar:fullpanels`) is used only by
  `pedroPathing/Tuning.java` for practice tuning.
- G403: TeleOp `init()` must not move any actuator.
- G407 / G408: hold at most 4 elements, and never hold opponent-color NECTAR.
- G402: AUTO stays on our side of the field (red = columns A–C, blue = D–F).
- R105.B: extension limits must be mechanical. Software limits don't count.

## Layout

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`: `auto/`, `teleop/`,
`subsystems/` (Shooter, Intake, and Arm are still empty placeholders),
`vision/` (Limelight ball detection, grouping, persistence), `control/` (ball
alignment PID), `pedroPathing/`, `hardware/RobotHardware.java`,
`util/Constants.java`. Unit tests are under `TeamCode/src/test/`.

Hardware-map names: `docs/HARDWARE_CONFIGURATION.md`. OpModes:
`docs/OPMODES.md`. Pedro: `docs/PEDRO_PATHING.md`.

Run unit tests with `./gradlew :TeamCode:testDebugUnitTest`.
