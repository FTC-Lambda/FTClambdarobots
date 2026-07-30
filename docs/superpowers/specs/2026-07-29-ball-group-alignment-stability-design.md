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
- With multiple groups, acquisition prefers the greatest ball count, then apparent closeness, then confidence, then proximity to the camera center.
- A different group cannot take an existing lock until it remains the preferred challenger for three consecutive fresh frames.
- A one-frame change in a locked group's reported ball count or position neither changes target identity nor exposes the raw position jump directly to the controller.
- Ball-specific tuning does not change AprilTag tracking behavior.

## Architecture

The feature keeps the existing perception-to-hardware flow but gives the control stage a dedicated boundary:

`Limelight detections -> BallGrouping -> TargetPersistence -> BallTarget -> BallAlignmentController -> Drivetrain`

`LimelightVision` continues to own detection parsing, filtering, grouping, target arbitration, and target persistence. `BallTarget.getHorizontalErrorDeg()` becomes the only bearing consumed by alignment control. `BallGroupingAligningTest` remains the application and behavioral layer: it polls vision, handles the existing `MANUAL`/`ALIGN` state machine and color controls, passes targets to the controller, applies the returned turn command through `Drivetrain`, and publishes telemetry. A separate `Ball-Group Alignment Tuner` Test OpMode uses the same production controller and vision pipeline while allowing an operator to adjust runtime configurations from Driver Station controls.

A new FTC-independent `BallAlignmentController` owns the control state. It composes the existing `PIDController` and `SlewRateLimiter` utilities but uses ball-specific configuration. It also owns correction hysteresis, fresh-frame command expiry, sign-reversal handling, and reset behavior. Keeping this class free of FTC SDK types allows deterministic local unit tests.

## Ball-Specific Configuration

Add descriptive default constants without changing the existing `VISION_SEEK_*` values used by AprilTag OpModes. The production OpMode constructs immutable runtime configuration snapshots from these defaults; the tuner constructs editable snapshots with the same initial values. Editing a tuner value never mutates a global constant or changes another OpMode.

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
- `BALL_TARGET_SWITCH_CONFIRM_FRAMES = 3`
- `BALL_TARGET_AIM_FILTER_GAIN = 0.50`

These are safe initial field-tuning values, not universal drivetrain calibration. The tuner displays every active value so the final verified set can be copied into the default constants deliberately after a field session; it does not persist changes across OpMode restarts.

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

## Multi-Group Arbitration

### Initial acquisition

After applying the requested color filter, target acquisition ranks groups lexicographically:

1. greater ball count;
2. greater apparent closeness, measured by the largest member's target area;
3. greater average confidence;
4. smaller absolute weighted horizontal error.

This means a three-ball group is preferred over a nearer two-ball group. Apparent closeness decides only when ball counts match. The existing desired-color behavior remains an eligibility filter: when at least one group contains the desired color, groups without that color are excluded; when no group contains it, acquisition falls back to all groups.

### Maintaining a lock

Every fresh frame first searches for groups inside the existing weighted `tx`/`ty` association gate. If more than one group is inside the gate, the group with the smallest angular distance from the previous lock is the current observation; apparent closeness, confidence, and center proximity break exact distance ties in that order. Other groups remain eligible challengers rather than silently replacing the lock. The selected current observation remains the current target even if its detected member count changes. Its current raw group is retained for diagnostics, while the `BallTarget` aimpoint is updated with an exponential filter:

`filteredAim = previousFilteredAim + 0.50 * (rawWeightedAim - previousFilteredAim)`

The filter is seeded directly from the first observation after acquisition or a confirmed switch. It smooths one-frame membership and detector-position jumps without blending the positions of two different locks.

### Challenging a lock

Any group not chosen as the current observation becomes a challenger only when it outranks that observation using the acquisition ordering. When ball counts tie, it must also exceed the current group's apparent closeness by the existing `CLOSEST_AREA_SWITCH_RATIO` of 1.25. A challenger must remain associated with the pending challenger and remain preferred for three consecutive fresh frames before it replaces the lock.

If challenger identity changes, it stops outranking the current group, or the current group regains priority, the confirmation count resets. Candidate ball-count fluctuations do not reset confirmation as long as the candidate remains associated and preferred.

If no current observation is associated in a fresh frame, the best acquisition-ranked group may still enter the same three-frame confirmation process. Until confirmation completes, persistence retains the previous filtered aimpoint as held rather than marking the challenger fresh. The alignment controller's 100 ms command timeout therefore stops motion while identity is uncertain. If no challenger completes confirmation before the normal 350 ms vision-loss timeout, the lock clears.

On the third consecutive confirming frame, the challenger becomes the lock, aimpoint filtering is reseeded from that group's weighted center, and the new target is marked fresh.

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

### `BallGroupAlignmentTuner`

- Is a separate `@TeleOp` in the Test group named `Ball-Group Alignment Tuner`.
- Uses the same `LimelightVision`, `TargetPersistence`, `BallAlignmentController`, and `Drivetrain` path as `BallGroupingAligningTest`; it does not duplicate control logic.
- Maintains editable `BallAlignmentConfig` and `BallTargetingConfig` instances seeded from `Constants` and `BallVisionConfig` defaults.
- Applies a new alignment configuration by calling `BallAlignmentController.setConfig(...)`, which safely resets transient PID, deadband, command, and slew state. Applies a new target configuration through `LimelightVision`, which clears the current target lock and pending challenger before reacquiring.
- Never writes preferences, files, or static constants. A tuning session therefore cannot accidentally change autonomous or AprilTag behavior.

### Runtime configuration objects

- `BallAlignmentConfig` contains PID gains, derivative filter, start/stop deadbands, maximum turn power, static-friction power, slew rate, and command-hold timeout.
- `BallTargetingConfig` contains challenger confirmation-frame count and aim-filter gain.
- Each configuration validates finite values and bounds them before it reaches a controller: nonnegative gains/powers/rates, `0 < stopDeadband <= startDeadband`, `0 < maxTurnPower <= 1`, `0 <= staticFrictionPower <= maxTurnPower`, positive command hold, confirmation frames in `[1, 10]`, and filter gain in `(0, 1]`.

### Existing vision classes

- `BallGrouping` continues grouping detections using spatial adjacency and the horizontal span cap.
- `BallGrouping` exposes one deterministic comparator for acquisition ordering: ball count, apparent closeness, confidence, then center proximity.
- `TargetPersistence` owns the locked group, pending challenger identity/count, three-frame confirmation, and filtered weighted aimpoint.
- New regression tests cover membership changes, association boundaries, initial ranking, challenger confirmation/reset, missing-lock behavior, area-ratio switching, and filtered weighted-aimpoint consistency.
- Grouping proximity thresholds are unchanged; this work changes arbitration and persistence after groups have already been formed.

## Telemetry

The alignment OpMode reports:

- controller action/reason;
- target state: fresh, held with age, or invalid;
- canonical weighted bearing used by control;
- raw and filtered locked-group bearings;
- group arithmetic mean as diagnostic data only;
- locked and pending group sizes;
- challenger confirmation count out of three;
- correction active/centered state;
- PID P/I/D terms;
- raw PID, requested, and final applied turn commands;
- whether static-friction compensation, slew limiting, command expiry, or reversal guarding affected the output;
- existing detection, rejection, grouping, color, pipeline, staleness, and latency data.

This separates perception instability from controller behavior during field tuning.

## Driver Station Tuning OpMode

`Ball-Group Alignment Tuner` provides an operator-controlled tuning loop without any external dashboard dependency.

### Controls

- `A`: enter ALIGN; `B`: return to MANUAL and command zero.
- `X`: prefer green; `Y`: prefer purple; either bumper: clear color preference.
- D-pad up/down: select the previous/next parameter.
- D-pad left/right: decrease/increase the selected parameter by its fine increment.
- Hold the left-stick button while pressing D-pad left/right: use the selected parameter's coarse increment.

The selectable parameters and increments are:

| Parameter | Fine | Coarse |
| --- | ---: | ---: |
| turn kP | 0.002 | 0.010 |
| turn kI | 0.0005 | 0.0020 |
| turn kD | 0.001 | 0.005 |
| derivative filter | 0.05 | 0.20 |
| start deadband (deg) | 0.25 | 1.00 |
| stop deadband (deg) | 0.25 | 1.00 |
| maximum turn power | 0.05 | 0.10 |
| static-friction power | 0.01 | 0.05 |
| turn slew rate | 0.5 | 2.0 |
| command hold (ms) | 25 | 100 |
| switch confirmation frames | 1 | 1 |
| aim-filter gain | 0.05 | 0.20 |

Changing either deadband clamps the other as needed to maintain `0 < stop <= start`. Changing maximum turn power clamps static-friction power if necessary. Every other value is constrained by the configuration validation rules.

### Display and safe application

Driver Station telemetry always shows the selected parameter, fine/coarse mode, all current configuration values, and the default value for the selected parameter. It also shows the full alignment telemetry: target freshness and age, lock/pending challenger state, raw/filtered bearing, correction state, PID terms, and raw/requested/applied power.

Every button action is edge-triggered. Applying a changed alignment configuration resets the alignment controller and outputs zero for that loop. Applying a changed targeting configuration clears the current target and pending challenger, then reacquires normally. The tuning OpMode uses the same immediate stop behavior as the alignment test when target visibility is lost, the operator exits ALIGN, or the OpMode ends.

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
- initial acquisition chooses greater ball count before apparent closeness;
- equal-count acquisition chooses greater apparent closeness before confidence and center proximity;
- persistence retains an associated group through one-frame member-count and position changes;
- a challenger cannot switch the lock before three consecutive confirming fresh frames;
- challenger identity or priority loss resets its confirmation count;
- an equal-count challenger must exceed the configured area switch ratio;
- a missing current observation holds the old target while a challenger confirms and does not authorize motion beyond 100 ms;
- filtered aimpoint state resets rather than blending two different group identities;
- the controller consumes the filtered persisted weighted bearing when arithmetic, raw weighted, and filtered centers differ.
- configuration validation clamps invalid relationships between deadbands and power limits;
- changing an alignment configuration resets the controller to zero;
- changing a targeting configuration clears lock and challenger state;
- the tuner parameter-selection and fine/coarse adjustment logic changes only the intended configuration field.

### Repository verification

Run the focused vision/control unit tests, the complete TeamCode unit-test suite, and a TeamCode debug compilation. Existing AprilTag tests and compilation must remain unchanged because their constants and controller construction are not modified.

### On-robot verification

1. Measure the smallest raw turn power that reliably rotates all four wheels on the actual surface; adjust only `BALL_ALIGN_STATIC_FRICTION_POWER`.
2. Place a stationary ball group approximately 15 degrees off center and verify prompt rotation without exceeding the 0.65 cap.
3. Reduce `BALL_ALIGN_TURN_KP` if the robot crosses center repeatedly; increase it only if large-error response remains too slow.
4. Increase `BALL_ALIGN_TURN_KD` in small increments only if proportional control overshoots despite correct deadband and command expiry.
5. Verify that a centered stationary target produces sustained zero output despite detector jitter.
6. Occlude the target and verify that commanded motion begins decaying after 100 ms and reaches immediate zero when the target becomes invalid.
7. Present groups with different member counts and verify the greater-count group wins acquisition even when the smaller group appears closer.
8. Present two equal-count groups and verify the apparently closer group wins acquisition.
9. Temporarily change a locked group's detected count or position and verify the lock remains stable.
10. Make a challenger preferable for fewer than three frames and verify it cannot steal the lock; sustain its advantage for three fresh frames and verify one deliberate switch.
11. In `Ball-Group Alignment Tuner`, adjust each parameter and verify Driver Station telemetry reflects the new runtime value, the alignment controller safely resets after an alignment setting changes, and the target lock safely reacquires after a targeting setting changes.

## Out of Scope

- Autonomous approach or range estimation from ball area.
- Changes to the Limelight neural model or pipeline configuration.
- Changes to AprilTag control gains or behavior.
- IMU-based heading hold or sensor fusion.
- Refactoring unrelated drivetrain, hardware, or OpMode code.
