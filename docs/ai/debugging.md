# AI debugging guide

This guide helps agents and developers diagnose FTC robot failures safely. It is for software investigation and robot test planning; it does not prove physical behavior.

## Evidence levels

Use these labels when making claims:

| Level | Meaning |
| --- | --- |
| Code-confirmed | Proven from source code. |
| Build-confirmed | Verified by Gradle or compiler output. |
| Telemetry-supported | Supported by robot telemetry. |
| Human-observed | Reported by a driver or team member. |
| Unconfirmed hypothesis | Plausible but not tested. |

Do not claim a robot behavior is fixed until the robot has been physically tested.

## Diagnosis rules

1. Start from the observed symptom, not the guessed cause.
2. Inspect relevant code before changing it.
3. Gather telemetry before tuning.
4. Change one variable at a time.
5. Keep motor outputs clamped.
6. Keep autonomous loops stop-aware and timeout-aware.
7. Never add blocking sleeps inside active robot control loops.
8. Never remove safety checks to test a theory.
9. Do not refactor unrelated code during debugging.

## Standard debug report

```text
Symptom:
- ...

Likely causes:
1. ...
2. ...
3. ...

Evidence level:
- Code-confirmed / Build-confirmed / Telemetry-supported / Human-observed / Unconfirmed hypothesis

Telemetry required:
- ...

Smallest diagnostic change:
- ...

Expected outcome:
- ...

If unsuccessful, next step:
- ...
```

## Common drivetrain failures

Symptoms:

- Robot rotates while commanded forward.
- Robot strafes diagonally.
- Robot drives backward.
- One side moves faster.
- Mecanum drive feels inconsistent.

Likely causes:

- Hardware names swapped.
- Motor directions wrong.
- Mecanum wheel orientation wrong.
- Joystick axes wrong.
- Motor scale constants wrong.
- Weak motor, slipping wheel, or wiring issue.

Check pure forward, pure strafe, and pure turn separately at low power. Log all four motor powers and compare commanded direction to physical motion.

## PID oscillation

Likely causes:

- `kP` too high.
- `kD` too low or too noisy.
- Integral windup.
- Wrong error sign.
- Output clamp too high.
- Inconsistent loop timing.
- Noisy or stale vision readings.

Tune with `kI = 0` first, start with low `kP`, clamp output, and log target, current value, error, PID output, clamped output, and loop `dt`.

## Autonomous failures

Likely causes:

- State transition never met.
- Timeout missing.
- Stop request not checked.
- Vision result is null or stale.
- Motors do not stop on exit.
- Units or target pose are wrong.

Every autonomous state should report state name, elapsed state time, transition inputs, timeout status, and failure reason.

## Vision and AprilTag failures

Likely causes:

- Wrong Limelight pipeline.
- Wrong accepted tag ID.
- Wrong tag size.
- Null or stale detections.
- Bearing, yaw, range, or units misunderstood.
- Output signs reversed.
- Camera mount or lighting issue.

Log pipeline, target tag ID, detected tag ID, detection age, yaw or bearing, range, selected target, state, and motor outputs. Test turn-only, strafe-only, and approach-only behavior before combining corrections.
