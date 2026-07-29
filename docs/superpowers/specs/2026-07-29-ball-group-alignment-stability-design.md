# Ball-Group Alignment Stability Design

## Goal

Make `Ball-Grouping-Aligning test` rotate promptly toward the persisted ball group, settle without continuous left-right hunting, and stop safely when vision data is no longer actionable.

## Success Criteria

- A visible target more than 4 degrees off center starts a correction.
- Once the target reaches 2 degrees or less from center, the controller commands zero turn and remains centered until a fresh measurement exceeds 4 degrees.
- Large errors may command up to 0.65 turn power.
- A repeated vision frame cannot keep the last nonzero command alive for more than 100 ms after the last fresh frame.
- An invalid target, manual-mode transition, color-preference change, or OpMode shutdown immediately commands zero and clears all controller state.
- PID, deadband, output limiting, target persistence, and telemetry all use the same persisted weighted horizontal error.
- Ball-specific tuning does not change AprilTag tracking behavior.

## Architecture

The feature keeps the existing perception-to-hardware flow but gives the control stage a dedicated boundary:

`Limelight detections -> BallGrouping -> TargetPersistence -> BallTarget -> BallAlignmentController -> Drivetrain`

`LimelightVision` continues to own detection parsing, filtering, grouping, and target persistence. `BallTarget.getHorizontalErrorDeg()` becomes the only bearing consumed by alignment control. `BallGroupingAligningTest` remains the application and behavioral layer: it polls vision, handles the existing `MANUAL`/`ALIGN` state machine and color controls, passes targets to the controller, applies the returned turn command through `Drivetrain`, and publishes telemetry.

A new FTC-independent `BallAlignmentController` owns the control state. It composes the existing `PIDController` and `SlewRateLimiter` utilities but uses ball-specific configuration. It also owns correction hysteresis, fresh-frame command expiry, sign-reversal handling, and reset behavior. Keeping this class free of FTC SDK types allows deterministic local unit tests.

## Ball-Specific Configuration

Add descriptive constants without changing the existing `VISION_SEEK_*` values used by AprilTag OpModes:

- `BALL_ALIGN_TURN_KP = 0.040`
- `BALL_ALIGN_TURN_KI = 0.0`
- `BALL_ALIGN_TURN_KD = 0.003`
- `BALL_ALIGN_TURN_DERIVATIVE_FILTER = 0.20`
- `BALL_ALIGN_START_CORRECTION_DEG = 4.0`
- `BALL_ALIGN_STOP_CORRECTION_DEG = 2.0`
- `BALL_ALIGN_MAX_TURN_POWER = 0.65`
- `BALL_ALIGN_STATIC_FRICTION_POWER = 0.12`
- `BALL_ALIGN_TURN_SLEW_RATE = 6.0` power units per second
- `BALL_ALIGN_COMMAND_HOLD_MS = 100`

These are safe initial field-tuning values, not universal drivetrain calibration. The final on-robot procedure changes only these ball-alignment constants if the measured drivetrain breakaway power or response requires adjustment.

## Control Behavior

### Fresh target

On each fresh `BallTarget`, the controller reads `getHorizontalErrorDeg()` and updates correction hysteresis:

- When inactive, correction starts only if `abs(error) > 4.0`.
- When active, correction stops when `abs(error) <= 2.0`.
- Entering the centered state immediately returns zero, resets PID history, and resets the slew limiter. This prevents residual slew output from carrying the robot through center.

While correction is active, PID is evaluated once using the elapsed time since the previous fresh frame. Integral gain is zero for the initial implementation because vision centering has no demonstrated steady-state bias that warrants accumulated error.

The raw PID output is clamped to `[-0.65, 0.65]`. Static-friction compensation raises a nonzero command whose magnitude is below `0.12` to `0.12`, using the PID output sign. Compensation is never applied while centered or while stopping.

### Repeated or held target

PID and deadband state are not updated from repeated measurements. For the first 100 ms after a fresh target, the controller may continue applying the last fresh requested command through the slew limiter. After 100 ms, the requested command becomes zero and the slew limiter decelerates the output toward zero. Target persistence may continue reporting the group for association and telemetry, but it cannot authorize sustained motion beyond this shorter control timeout.

### Direction reversal

If a fresh requested command has the opposite sign from the currently applied command, the controller outputs zero for that update and resets the slew limiter. It does not immediately drive in the opposite direction. A subsequent fresh frame must confirm that the opposite correction is still required before motion restarts. This zero-crossing guard prevents delayed vision noise from driving a full left-to-right reversal.

### Invalid target and reset events

An invalid target returns zero immediately rather than slewing down. It also clears PID history, hysteresis, cached commands, timing state, and the slew limiter. The same reset is used when entering or leaving alignment, changing color preference, and shutting down the OpMode.

All elapsed-time inputs are sanitized. Non-finite, negative, or near-zero loop intervals produce a safe zero step or reset rather than an unbounded derivative or slew calculation.

## Component Responsibilities

### `BallAlignmentController`

- Consumes a `BallTarget`, current monotonic time in milliseconds, and control-loop `dt`.
- Produces an immutable result containing final turn command, raw PID command, requested command, canonical bearing, correction state, and an action/reason enum.
- Updates PID only for fresh targets.
- Enforces hysteresis, command expiry, static-friction compensation, clamping, slew limiting, reversal confirmation, and immediate safe stop.
- Exposes `reset()` for every behavioral transition.

### `BallGroupingAligningTest`

- Retains controller/operator state and drivetrain ownership.
- Replaces direct PID/deadband/slew manipulation with one controller call.
- Uses the controller result for motor output and telemetry.
- Keeps `BallGroup` data only for diagnostic group membership and area telemetry; it does not derive a second control bearing.

### Existing vision classes

- `BallGrouping` continues grouping detections using spatial adjacency and the horizontal span cap.
- `TargetPersistence` continues associating the current group and publishing the current frame's weighted aimpoint.
- New regression tests cover membership changes, association boundaries, area-ratio switching, and weighted-aimpoint consistency.
- No grouping threshold or selection algorithm changes are included unless a deterministic regression test first demonstrates that the current implementation violates these documented rules.

## Telemetry

The alignment OpMode reports:

- controller action/reason;
- target state: fresh, held with age, or invalid;
- canonical weighted bearing used by control;
- group arithmetic mean as diagnostic data only;
- correction active/centered state;
- PID P/I/D terms;
- raw PID, requested, and final applied turn commands;
- whether static-friction compensation, slew limiting, command expiry, or reversal guarding affected the output;
- existing detection, rejection, grouping, color, pipeline, staleness, and latency data.

This separates perception instability from controller behavior during field tuning.

## Error Handling and Safety

- Manual mode, target loss, controller reset, and OpMode shutdown always have an immediate zero-output path.
- Held targets degrade to zero after 100 ms instead of treating the 350 ms persistence window as permission to move.
- Output remains clamped before reaching `Drivetrain`.
- The existing drivetrain hardware abstraction remains the only class that writes motor power.
- The control loop remains non-blocking and performs no sleeps or waits for vision.

## Verification

### Local unit tests

Pure-Java tests will verify:

- correction starts above 4 degrees and remains inactive at or below 4 degrees;
- correction stops at or below 2 degrees and does not restart within the hysteresis band;
- output never exceeds 0.65;
- static-friction compensation is applied only during active correction;
- repeated frames do not recalculate PID;
- held commands expire at 100 ms and decay toward zero;
- invalid targets and explicit reset return zero immediately;
- a sign reversal produces zero and requires a confirming fresh frame;
- controller state does not survive mode or color resets;
- grouping remains bounded by maximum span;
- persistence retains an associated group unless the closest alternative exceeds the configured area switch ratio;
- the controller consumes the weighted persisted bearing when arithmetic and weighted group centers differ.

### Repository verification

Run the focused vision/control unit tests, the complete TeamCode unit-test suite, and a TeamCode debug compilation. Existing AprilTag tests and compilation must remain unchanged because their constants and controller construction are not modified.

### On-robot verification

1. Measure the smallest raw turn power that reliably rotates all four wheels on the actual surface; adjust only `BALL_ALIGN_STATIC_FRICTION_POWER`.
2. Place a stationary ball group approximately 15 degrees off center and verify prompt rotation without exceeding the 0.65 cap.
3. Reduce `BALL_ALIGN_TURN_KP` if the robot crosses center repeatedly; increase it only if large-error response remains too slow.
4. Increase `BALL_ALIGN_TURN_KD` in small increments only if proportional control overshoots despite correct deadband and command expiry.
5. Verify that a centered stationary target produces sustained zero output despite detector jitter.
6. Occlude the target and verify that commanded motion begins decaying after 100 ms and reaches immediate zero when the target becomes invalid.
7. Present two similarly sized groups and verify persistence does not alternate targets frame to frame.

## Out of Scope

- Autonomous approach or range estimation from ball area.
- Changes to the Limelight neural model or pipeline configuration.
- Changes to AprilTag control gains or behavior.
- IMU-based heading hold or sensor fusion.
- Refactoring unrelated drivetrain, hardware, or OpMode code.
