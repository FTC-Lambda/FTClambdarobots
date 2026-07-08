# AI Architecture — Lambda Robotics

This file collects high-level architecture and AI-relevant notes for developer agents and humans. Source-of-truth references: project `README.md`, `TeamCode` packages, and the AprilTag design spec.

- **Purpose:** summarize package layout, subsystems, control and command flow, vision pipeline, and autonomous structure for AI-first workflows.

- **Package layout (key folders):**
	- `TeamCode/src/main/java/org/firstinspires.ftc.teamcode/auto` — autonomous OpModes
	- `TeamCode/src/main/java/org/firstinspires.ftc.teamcode/teleop` — teleop OpModes
	- `TeamCode/src/main/java/org/firstinspires.ftc.teamcode/hardware` — `RobotHardware.java` (hardware map)
	- `TeamCode/src/main/java/org/firstinspires.ftc.teamcode/subsystems` — `Drivetrain`, `Arm`, `Intake`, `Shooter`
	- `TeamCode/src/main/java/org/firstinspires.ftc.teamcode/util` — constants and helpers

- **Subsystems (confirmed):**
	- `Drivetrain` — movement + odometry
	- `Arm` — arm mechanism
	- `Intake` — game piece intake
	- `Shooter` — shooting mechanism
	- TODO: any additional subsystems — check `subsystems` folder for names and responsibilities

- **Control flow / command flow:**
	- TeleOp OpModes instantiate `RobotHardware` and subsystem objects and call subsystem `drive()` / control methods each loop.
	- Autonomous OpModes are implemented as `LinearOpMode` or iterative OpModes that sequence phases (see AprilTag specs).
	- TODO: list command scheduler or pattern if present (e.g., command-based) — search codebase for scheduler usage.

- **Vision pipeline:**
	- Project uses Limelight 3A (AprilTag pipeline referenced in specs and plans).
	- Camera configured in hardware as `limelight` (see AprilTag spec). Pipeline 0 expected to be an AprilTag pipeline.
	- Vision consumers: `AprilTagSeekAuto` (design/spec), other OpModes may reference Limelight classes.
	- TODO: confirm exact Limelight Java classes and calls by inspecting `hardware/` and `auto/` code.

- **Autonomous structure (example from AprilTag spec):**
	- Phased `LinearOpMode`: SPIN → CENTER → APPROACH
	- Uses `Drivetrain.driveRaw(...)` for raw motor control (plan calls for adding this method).
	- Tunable constants should be `static final` at top of OpMode classes.

- **How AI agents should use these docs:**
	- Read `RobotHardware.java` for authoritative hardware names and ports before proposing code changes.
	- Reuse existing spec/plan content (migrated here or referenced) rather than duplicating.
	- Leave TODO placeholders where hardware or parameters are unknown.

References:
- `README.md` — project intro, subsystem list, motor calibration procedure
- `docs/superpowers/specs/2026-06-24-apriltag-seek-auto-design.md` — AprilTag autonomous design (migrated content should be condensed here)
- `docs/superpowers/plans/2026-06-24-apriltag-seek-auto.md` — implementation plan (use for developer tasks)

