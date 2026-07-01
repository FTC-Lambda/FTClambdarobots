# AprilTag Seek Autonomous — Design Spec
**Date:** 2026-06-24
**Branch:** An2468/cambridge

## Overview

A new autonomous OpMode (`AprilTagSeekAuto`) that spins the robot in place, detects the first visible AprilTag, centers on it, and drives toward it. Implemented as a single `LinearOpMode` with three sequential phases.

## Goals

- Spin up to 360° to scan for AprilTags
- Lock onto the first detected tag
- Keep spinning until that tag is centered in frame (bearing ≈ 0°)
- Drive toward the tag and stop at a configurable distance (~12 inches)

## Files Changed

| File | Change |
|------|--------|
| `TeamCode/.../auto/AprilTagSeekAuto.java` | New — main OpMode |
| `TeamCode/.../hardware/RobotHardware.java` | Add `Limelight3A limelight` field |

## Architecture

Single `LinearOpMode`. Uses existing `RobotHardware` (motors) and `Drivetrain`. Adds `Limelight3A` for AprilTag detection. Phase state tracked by a local `enum Phase { SPIN, CENTER, APPROACH }` inside the OpMode.

Camera: Limelight 3A connected to Control Hub USB 3.0, hardware config name `"limelight"`.

## Phase Detail

### Phase 1 — SPIN
- Drive: `drivetrain.drive(0, 0, SPIN_POWER)`
- Poll `limelight.getLatestResult()` every loop iteration
- First frame with any AprilTag detection: lock `lockedTagId`, transition to CENTER
- Safety timeout: if `SPIN_TIMEOUT_MS` elapses with no detection, stop and end OpMode

### Phase 2 — CENTER
- Continue rotating (same direction, same power) while polling for `lockedTagId`
- When the locked tag's bearing is within `BEARING_THRESHOLD` degrees of 0°: stop motors, transition to APPROACH
- Safety timeout: `CENTER_TIMEOUT_MS` — if tag is lost, stop and end OpMode

### Phase 3 — APPROACH
- Proportional control: `drive = rangeError * SPEED_GAIN`, `turn = bearing * TURN_GAIN`
- Powers clamped to `MAX_DRIVE_SPEED` and `MAX_TURN_SPEED`
- Stop condition: `range ≤ DESIRED_DISTANCE` OR tag lost

## Tunable Constants

```java
static final double SPIN_POWER        = 0.3;
static final long   SPIN_TIMEOUT_MS   = 3000;
static final double BEARING_THRESHOLD = 5.0;   // degrees
static final long   CENTER_TIMEOUT_MS = 2000;
static final double DESIRED_DISTANCE  = 12.0;  // inches
static final double SPEED_GAIN        = 0.02;
static final double TURN_GAIN         = 0.01;
static final double MAX_DRIVE_SPEED   = 0.4;
static final double MAX_TURN_SPEED    = 0.3;
```

## Out of Scope

- IMU / gyro tracking
- Choosing between multiple detected tags (first seen wins)
- Any action after reaching the tag (e.g. scoring)
