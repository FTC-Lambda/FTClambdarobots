# What BIOBUZZ means for this codebase

This repo was written against DECODE (2025–26) assumptions: green and purple
balls, AprilTags 20–24, 165.1 mm tags, SDK 11.1. This file lists every place
where that conflicts with BIOBUZZ (Competition Manual TU08), ordered by how much
it matters. Game facts are in [BIOBUZZ.md](BIOBUZZ.md).

Nothing below has been changed in code yet. Each item says what to change.

---

## P0 — Legality: fix before any event

### 1. Panels telemetry is prohibited at competition (R704.D)

R704.D: *"Additional logging/streaming services, such as those hosted by third
party plugins and tools such as FTC Dashboard, FTControl Panels, and others are
prohibited."*

This repo uses FTControl Panels:
- `build.dependencies.gradle`: `com.bylazar:fullpanels:1.0.12` and
  `com.pedropathing:telemetry:1.0.0`
- `TeamCode/.../pedroPathing/Tuning.java`: `PanelsTelemetry`,
  `PanelsConfigurables`, `PanelsField`

Panels is a library bundled into the Robot Controller app. Assume it serves
data whenever the app is installed, not only while `Tuning` is running.
Recommended approach: keep Panels for practice tuning, but ship a competition
build without it. That could be a Gradle product flavor, or simply removing the
dependency and `Tuning.java` from the branch you deploy at events. If the rule's
scope is unclear for your setup, ask on the official Q&A
(https://ftc.game/qa).

## P1 — Wrong game data in code

### 2. AprilTag size: 165.1 mm → 82.55 mm (§9.9)

- `teleop/LimelightPipelineSetup.java:39`: `FIDUCIAL_SIZE_MM = 165.1`. The
  comment there correctly notes this is the FRC size.
- `auto/AprilTagSeekAuto.java:34`: `FIDUCIAL_SIZE_MM = 165.1`

BIOBUZZ tags are **3.25 in = 82.55 mm**. With the wrong size, the Limelight's 3D
distance estimate comes out **about 2× too large**. Move the value into
`util/Constants.java` so it exists in one place, and update
`docs/HARDWARE_CONFIGURATION.md`, which also says 165.1 mm.

**Done:** `Constants.APRILTAG_SIZE_MM = 82.55` is now used by `LimelightPipelineSetup`,
`AprilTagSeekAuto` and the HIVE OpModes, and the hardware doc is updated.

### 3. AprilTag IDs: 20–24 → 30–45 (§9.9)

- `teleop/LimelightSeekTag.java:52`: `TAG_CHOICES = {20, 21, 22, 23, 24}`.
  These are DECODE IDs. None of them exist on a BIOBUZZ field.
- `auto/AprilTagSeekAuto.java` locks onto whichever tag it sees first.

Replace these with alliance-aware constants: red CELLs use 30–33 (far) and
34–37 (audience); blue CELLs use 38–41 (audience) and 42–45 (far). Each CELL
carries a 4-tag cluster, so aim at the **cluster centroid**, or at whichever
tags in it are visible, rather than at a single ID. MegaTag-style multi-tag
solving helps with this.

### 4. Tag geometry: high, angled, and moving (inference from Figures 9-10 and 9-16)

The DECODE-era seek logic ("spin until a tag appears, center it, then drive to
12 in") assumes a tag at roughly camera height on a fixed wall. BIOBUZZ tags
sit on the ends of CELLs about 30–65 in up, and the HIVE rotates on every TIP.
Expect to:
- re-mount or re-pitch the Limelight upward, then measure and record its pose
  in the Limelight config;
- drop "approach to 12 in". The HIVE is a launch target, not something to
  drive up to. G417 forbids touching it to cause a TIP.
- work out which CELL is currently up from which tag IDs are visible and
  their vertical angle (`ty`).

Verify all of this against the Onshape field CAD before building on it.

#### HIVE state detection (implemented)

The last bullet above is now code in `vision/hive/`. `HiveDetector` reads Limelight pipeline 0,
turns each visible HIVE tag (IDs 30-45, `HiveTagMap`) into a height above the tiles from its
3D range and `tx`/`ty` plus the camera mount (`HiveGeometry`, `Constants.LIMELIGHT_HEIGHT_IN` and
`LIMELIGHT_PITCH_DEG`), and feeds `HiveStateEstimator`. The estimator takes the per-CELL median
height, classifies it RAISED/LOWERED with a threshold and hysteresis (`HiveDetectionConfig`),
and confirms a `HiveState` (`FAR_CELL_UP`, `AUDIENCE_CELL_UP`, `UNKNOWN`) over several frames.
Try it with the `Hive State Test` OpMode and measure the real heights with `Hive Height
Calibration` (see `docs/OPMODES.md`). The default threshold (40 in) and camera mount are
placeholders until measured on the robot and field.

### 5. Ball classes: GREEN/PURPLE → POLLEN (yellow) and NECTAR (red/blue)

- `vision/BallColor.java`: the enum is `GREEN, PURPLE, UNKNOWN`, matched from
  detector class names on pipeline 1.
- `teleop/BallVisionTest`, `BallGroupingAligningTest`, and
  `BallGroupAlignmentTuner` bind X to green and Y to purple
  (`docs/OPMODES.md`).
- `vision/BallVisionConfig.java`: area thresholds (`MIN_TARGET_AREA`,
  `SCORE_AREA_SATURATION`, "~three balls at 0.02") were tuned on DECODE-sized
  balls.

Needed changes:
- New classes: `POLLEN`, `NECTAR_RED`, `NECTAR_BLUE`. The Limelight neural
  detector on pipeline 1 has to be retrained or replaced with a model that
  knows BIOBUZZ elements. A yellow color pipeline is a quick fallback for
  POLLEN.
- **G408**: never target opponent-color NECTAR. Targeting must take the
  alliance color and filter out the other NECTAR color, not just "prefer" a
  color.
- Size cue: NECTAR (3.6 in) is about 1.3× POLLEN's (2.8 in) diameter, so area
  at a known range can help separate them. Retune all area constants.
- The grouping, persistence, and PID alignment stack (`BallGrouping`,
  `TargetPersistence`, `BallAlignmentController`) is game-agnostic and should
  carry over. Only the class model and the thresholds change.

## P2 — Mechanisms the game requires (placeholders today)

`subsystems/Shooter.java`, `Intake.java`, and `Arm.java` are empty classes.
BIOBUZZ needs them as follows:

| Subsystem | BIOBUZZ job | Software requirements |
| --- | --- | --- |
| **Shooter** (launcher) | LAUNCH into the upward-facing CELL (opening 53.5–65.6 in high, about 20 × 14 in). This is the only way to TIP (G417). | Flywheel velocity control: PIDF plus feedforward, a ready-to-fire gate, voltage compensation, and range-based speed or hood angle from odometry or tag distance. Pause if a TIP is in progress (§10.5.1 note). |
| **Intake** | Floor pickup, plus pulling POLLEN from a FLOWER's bottom retrieval opening (≈ 3.55 in tall). | **Hard cap of 4 held** (G407). Count with beam-break or color/distance sensors and stop or reverse the intake at 4. Reject opponent NECTAR (G408). The start pose must touch exactly 4 POLLEN (G304). |
| **Arm / placer** | Put NECTAR and POLLEN into a FLOWER's top (≈ 4 in opening, 21.5 in high), legal only after 1:00 (G410). | Repeatable preset positions and a lockout before 1:00. Extension limits must be **mechanical** (R105.B), so software soft limits are just a convenience. |

Budget: 8 motors + 8 servos, 2 per port (R503 / R505). Drive uses 4 motors.

## P3 — OpMode behavior to add

- **Alliance selection** in INIT for every AUTO and for TeleOp. Tag IDs,
  NECTAR color, the field half (A–C red vs D–F blue), and the GARDEN all
  depend on it.
- **AUTO baseline** (30 s): fire the 4 preloads into the CELL, LEAVE the wall,
  and end partly inside the LOADING ZONE. That's 3 + 5 points plus any TIP at
  20, and it feeds the SWARM RP (≥ 16 LEAVE + PARK points per alliance).
  **Stay on your side** (G402). `BasicAuto.java` is empty, and
  `AprilTagSeekAuto` does not score.
- **TeleOp INIT must not move actuators** (G403). Check `MainTeleOp`'s
  `init()` once the subsystems exist.
- **Match-time cues** in TeleOp: rumble or telemetry at **1:00** (FLOWERS and
  all NECTAR open) and **0:20** (start heading to the LOADING ZONE). See the
  `ConceptGamepadRumble` sample.
- **Pedro paths and poses**: rebuild them from the BIOBUZZ field. The HIVE
  frame at center (49.46 × 38.95 in base) is a large obstacle, and FLOWERS sit
  on the walls.

## P4 — Tooling and housekeeping

- **SDK 11.1 → 12.0.** `FIRST-Tech-Challenge/FtcRobotController` v12.0
  (2026-09-12) is "the official release for the 2026/2027 season." It needs
  **Android Studio Narwhal 3 Feature Drop** or newer; the README says
  Ladybug. Bump the eight `org.firstinspires.ftc:*:11.1.0` artifacts and merge
  the upstream `FtcRobotController` module. Check that Pedro 2.1.2 still
  builds afterwards.
- `README.md` links to `docs/ai/hardware.md`, `docs/ai/pedro-pathing.md`, and
  `docs/ai/opmodes.md`, which don't exist. The real files are `docs/*.md`.
- Device names must follow R705: `<team#>-RC` and `<team#>-DS`.
- The Limelight 3A is legal and reprogrammable (R702, Table 12-9). Don't
  upgrade to a Limelight 3G; it is listed as prohibited.
