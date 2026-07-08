# AI Debugging Guide — Lambda Robotics

Purpose: help AI agents and human developers diagnose FTC robot failures safely, without guessing or changing too much at once.

This guide is for software-level investigation. It does not replace real robot testing.

---

## AI Limitation

AI agents can inspect code, add telemetry, add diagnostic OpModes, run Gradle builds, review diffs, and create robot test procedures.

AI agents cannot directly verify physical robot behavior, including:

- motor direction on the real robot
- mecanum wheel orientation
- wiring correctness
- battery sag
- camera mounting angle
- mechanical looseness
- carpet traction
- real AprilTag stability
- whether the robot actually moves correctly

Physical conclusions require human-provided evidence:

- telemetry logs
- Driver Station observations
- robot videos
- hardware photos
- test results

---

## Evidence Levels

When diagnosing, label conclusions as:

- **Code-confirmed** — proven from source code.
- **Build-confirmed** — verified by Gradle/build output.
- **Telemetry-supported** — supported by robot telemetry.
- **Human-observed** — based on driver/team observation.
- **Unconfirmed hypothesis** — plausible but not yet tested.

Agents must not claim robot behavior is fixed until the robot has been physically tested.

---

## Diagnosis Rules

AI agents must diagnose before modifying code.

Rules:

1. Start from the observed symptom, not the user's guessed cause.
2. Inspect relevant code before proposing changes.
3. Gather telemetry before tuning.
4. Rank possible causes by probability and risk.
5. Suggest the smallest diagnostic test first.
6. Modify only one thing at a time.
7. Never remove safety checks to "see if it works."
8. Never increase motor power before understanding the failure.
9. Do not refactor unrelated code during debugging.
10. Do not merge untested drivetrain/autonomous changes directly to `main`.

---

## Standard Debug Workflow

```text
Observe symptom
        ↓
Inspect relevant subsystem
        ↓
Collect telemetry
        ↓
Compare with expected behavior
        ↓
Rank likely causes
        ↓
Perform one diagnostic change
        ↓
Retest
        ↓
Repeat if needed

Every debugging response should use this format:

AI Debug Report

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

## General FTC Checklist

Before debugging software, verify:

hardware config names match RobotHardware.java
emergency stop behavior works
Common Failure Guide
Robot Moves Incorrectly

Symptoms:

rotates while commanded forward
strafes diagonally
drives backward
turns wrong direction
one side moves faster
mecanum drive feels inconsistent

Likely causes:

motor directions wrong
hardware map names swapped
mecanum equation wrong
joystick axes wrong
field-centric transform wrong
IMU heading sign wrong
motor scale constants wrong
wheel orientation wrong
wires swapped
weak/slipping motor or wheel
uneven weight distribution

Check:

test each motor individually
confirm motor names match physical motors
verify mecanum wheel orientation
command pure forward, pure strafe, pure turn
log all four motor powers
test at low power first
Bad Strafing / Drift

Symptoms:

drifts while strafing
rotates during strafe
cannot strafe straight
strafes better one direction than the other

Likely causes:

motor scales wrong
motor directions inconsistent
wheel orientation wrong
input normalization wrong
rollers dirty/damaged
frame flex
weight imbalance
one motor weaker

Check:

verify wheel orientation
test robot-centric before field-centric
reduce speed and retest
compare all motor powers
check Constants.java
run motor calibration OpMode if available

Do not compensate for major mechanical problems using software.

PID Oscillation

Symptoms:

repeated overshoot
sustained shaking around target
robot sways rapidly left-right
motor output changes sign repeatedly
robot gets close but never settles

Likely causes:

kP too high
kD too low, missing, or too noisy
integral windup
wrong error sign
no deadband
output clamp too high
minimum power too high
inconsistent loop timing
degrees/radians mismatch
angle wraparound bug
stale/noisy vision
loose or slipping drivetrain parts

Check:

set kI = 0
start with low kP
clamp output safely
verify correction direction
log loop delta time
test heading-only first
test on blocks before carpet

Log:

target
current value
error
PID output
clamped output
loop dt
battery voltage
current OpMode state

Tune one constant at a time.

Autonomous Failures

Symptoms:

stuck in one phase
does nothing after start
never exits loop
drives forever
waits forever for vision
path drifts

Likely causes:

state transition never met
timeout missing
pose not updated
vision result null/stale
motors not stopped on exit
stop request not checked
target pose/units wrong
follower not updated
state variable not changed

Check:

log current state
log elapsed time in state
log transition conditions
log target pose/current pose
log latest vision result
confirm timeout for every state
confirm motors stop on failure/completion

Autonomous must always:

check stop requests
include timeouts
handle missing sensor data
stop motors on exit
fail safely
report failure reason through telemetry
Vision Loss / Poor Detection

Symptoms:

no AprilTag detections
intermittent detections
late detections
wrong target
robot uses stale data
oscillates while using vision

Likely causes:

wrong pipeline
null result not handled
stale detection reused
wrong tag ID accepted
misunderstood vision units
wrong camera result fields
poor lighting
tag too far/angled
wrong tag size
dirty lens
unstable USB/power

Check:

verify Limelight web UI detections
confirm active pipeline
confirm target tag ID
log raw vision results
log detection age/timestamp
test stationary first
test under match-like lighting
check camera mount stability

Required behavior:

null checks
stale-data rejection
detection timeout
fallback if target is lost
reduced speed during alignment
AprilTag Debugging

Symptoms:

wrong tag ID
incorrect range
incorrect yaw/bearing
robot turns away from tag
robot centers but approaches wrong
robot oscillates near tag

Likely causes:

wrong AprilTag pipeline
wrong accepted tag ID
camera angle mismatch
sign convention wrong
range/bearing fields misunderstood
degrees/radians mismatch
detection noisy near target
turn/strafe/forward combined too aggressively

Check:

log tag ID
log yaw / tx / bearing
log range / distance
log target error
log drive, strafe, and turn separately
test turn-only
test strafe-only
test approach-only
move tag left/right manually to confirm output sign

Safe state pattern:

SEARCH
↓ tag found
ALIGN_HEADING
↓ heading error small
ALIGN_STRAFE
↓ lateral error small
APPROACH
↓ range target reached
DONE

Each state needs:

timeout
null detection handling
stale detection handling
clamped output
telemetry
safe stop path
Encoder / IMU / Localization Issues

Encoder symptoms:

reads zero
jumps randomly
odometry diverges
distance wrong
motor position does not change

Check:

log raw counts
confirm motor mode
confirm encoder direction
move one wheel manually and watch telemetry
reseat cables
reset encoders only intentionally

IMU symptoms:

heading jumps
field-centric drive points wrong
robot turns opposite direction
heading resets unexpectedly

Check:

log raw heading
rotate robot by hand
test ±90 degrees
check wraparound near ±180 degrees
confirm units
confirm mounting orientation

Localization symptoms:

pose drifts
pose jumps
robot follows paths incorrectly
autonomous starts from wrong location

Check:

log current pose every loop
log target pose
verify pose update frequency
push robot by hand and watch pose
confirm starting pose before auto
Telemetry Checklist

For drivetrain/control issues, log:

battery voltage
loop dt
current state
gamepad inputs
drive / strafe / turn commands
individual motor powers
motor scale factors
heading
target heading
error
PID output
clamped output

For vision issues, log:

pipeline number
target tag ID
detected tag ID
detection valid/invalid
detection timestamp/age
yaw / tx / bearing
range / distance
selected target
state transition reason

For autonomous issues, log:

current state
elapsed time in state
target pose
current pose
transition condition values
timeout status
failure reason

Prefer short structured telemetry.

Example:

state=ALIGN tag=3 yaw=4.2 range=31.5 error=4.2 turn=0.12 dt=0.021
Never Do These

Never:

tune multiple PID constants at once
change multiple subsystems at once
remove safety checks
increase power before diagnosis
ignore null vision results
reuse stale vision indefinitely
add blocking sleeps inside active control loops
refactor unrelated code while debugging
assume the user's diagnosis is correct
claim the robot is fixed without physical testing