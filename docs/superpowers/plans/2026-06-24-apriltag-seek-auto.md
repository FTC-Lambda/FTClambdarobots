# AprilTag Seek Autonomous Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an autonomous OpMode that spins in place, locks onto the first AprilTag it sees, centers the robot on it, then drives to it.

**Architecture:** Three sequential phases (SPIN → CENTER → APPROACH) inside a single `LinearOpMode`. The Limelight 3A camera is queried each loop via the FTC SDK's `Limelight3A` hardware class. A `driveRaw` method is added to `Drivetrain` so autonomous code gets direct power control without the joystick squaring that `drive()` applies.

**Tech Stack:** FTC SDK (Java), Limelight 3A (`com.qualcomm.hardware.limelightvision`), existing `RobotHardware` + `Drivetrain` subsystems.

## Global Constraints

- Java, FTC SDK conventions (no traditional unit tests — verification is compilation + on-robot run)
- Do not modify `Drivetrain.drive()` — it is used by TeleOp and must stay unchanged
- Hardware config name for camera: `"limelight"` (must match robot config on Driver Station)
- Limelight pipeline 0 must be configured as an AprilTag pipeline in the Limelight web UI before running
- All tunable values must be `static final` constants at the top of the OpMode class — never hardcoded in logic
- Build command: `./gradlew :TeamCode:build` from repo root

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/RobotHardware.java` | Modify | Add `Limelight3A limelight` field and init |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drivetrain.java` | Modify | Add `driveRaw(y, x, rx)` for direct power (no squaring, no DRIVE_SPEED) |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/AprilTagSeekAuto.java` | Create | Full autonomous OpMode: SPIN → CENTER → APPROACH |

---

### Task 1: Add Limelight3A to RobotHardware and driveRaw to Drivetrain

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/RobotHardware.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drivetrain.java`

**Interfaces:**
- Produces: `RobotHardware.limelight` (`Limelight3A`) — used by AprilTagSeekAuto
- Produces: `Drivetrain.driveRaw(double y, double x, double rx)` — used by AprilTagSeekAuto

**Why driveRaw is needed:** `Drivetrain.drive()` squares inputs and multiplies by `Constants.DRIVE_SPEED = 0.5`. Passing `rx = 0.3` to `drive()` yields actual wheel power `0.3² × 0.5 = 0.045` — far too weak for reliable autonomous movement. `driveRaw` sends power directly to motors with only normalization.

- [ ] **Step 1: Add Limelight3A import and field to RobotHardware**

Open `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/RobotHardware.java`.

Add the import at the top:
```java
import com.qualcomm.hardware.limelightvision.Limelight3A;
```

Add the field after the motor declarations:
```java
public Limelight3A limelight;
```

Add initialization inside `init()` after the motor lines:
```java
limelight = hardwareMap.get(Limelight3A.class, "limelight");
```

Final file:
```java
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {

    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor topLeft;
    public DcMotor topRight;

    public Limelight3A limelight;

    public void init(HardwareMap hardwareMap) {
        backLeft  = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        topLeft   = hardwareMap.get(DcMotor.class, "top_left");
        topRight  = hardwareMap.get(DcMotor.class, "top_right");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }
}
```

- [ ] **Step 2: Add driveRaw to Drivetrain**

Open `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drivetrain.java`.

Add this method after the existing `drive()` method:

```java
public void driveRaw(double y, double x, double rx) {
    double frontLeft  =  y + x + rx;
    double backLeft   =  y - x + rx;
    double frontRight =  y - x - rx;
    double backRight  =  y + x - rx;

    double max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeft),
            Math.max(Math.abs(frontRight), Math.abs(backRight))));
    if (max > 1.0) {
        frontLeft  /= max;
        backLeft   /= max;
        frontRight /= max;
        backRight  /= max;
    }

    robot.topLeft.setPower(Math.max(-1.0, Math.min(1.0, frontLeft  * motorScales[MOTOR_TOP_LEFT])));
    robot.backLeft.setPower(Math.max(-1.0, Math.min(1.0, backLeft   * motorScales[MOTOR_BACK_LEFT])));
    robot.topRight.setPower(Math.max(-1.0, Math.min(1.0, frontRight * motorScales[MOTOR_TOP_RIGHT])));
    robot.backRight.setPower(Math.max(-1.0, Math.min(1.0, backRight  * motorScales[MOTOR_BACK_RIGHT])));
}
```

- [ ] **Step 3: Build and verify compilation**

```bash
./gradlew :TeamCode:build
```

Expected: `BUILD SUCCESSFUL`. Fix any import or syntax errors before continuing.

- [ ] **Step 4: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/RobotHardware.java \
        TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drivetrain.java
git commit -m "feat: add Limelight3A to RobotHardware and driveRaw to Drivetrain"
```

---

### Task 2: Create AprilTagSeekAuto — SPIN phase

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/AprilTagSeekAuto.java`

**Interfaces:**
- Consumes: `RobotHardware.limelight` (`Limelight3A`)
- Consumes: `Drivetrain.driveRaw(double y, double x, double rx)`
- Produces: `AprilTagSeekAuto` OpMode visible in Driver Station under Autonomous

- [ ] **Step 1: Create the file with constants, phase enum, and SPIN phase**

Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/AprilTagSeekAuto.java`:

```java
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.List;

@Autonomous(name = "AprilTag Seek", group = "Auto")
public class AprilTagSeekAuto extends LinearOpMode {

    // --- Tunable constants ---
    static final double SPIN_POWER         = 0.3;   // raw turn power during spin
    static final long   SPIN_TIMEOUT_MS    = 3000;  // give up if no tag found in this window
    static final double BEARING_THRESHOLD  = 5.0;   // degrees — "centered enough"
    static final long   CENTER_TIMEOUT_MS  = 2000;  // give up centering if tag lost
    static final double DESIRED_DISTANCE   = 12.0;  // inches — stop distance
    static final double SPEED_GAIN         = 0.02;  // forward proportional gain
    static final double TURN_GAIN          = 0.01;  // turn proportional gain
    static final double MAX_DRIVE_SPEED    = 0.4;   // cap on forward power
    static final double MAX_TURN_SPEED     = 0.3;   // cap on turn correction power

    private enum Phase { SPIN, CENTER, APPROACH, DONE }

    private RobotHardware robot;
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        drivetrain = new Drivetrain(robot);

        robot.limelight.pipelineSwitch(0); // Pipeline 0 must be AprilTag in Limelight web UI
        robot.limelight.start();

        telemetry.addData("Status", "Initialized — waiting for start");
        telemetry.update();
        waitForStart();

        Phase phase = Phase.SPIN;
        int lockedTagId = -1;
        ElapsedTime timer = new ElapsedTime();

        // --- SPIN phase ---
        timer.reset();
        while (opModeIsActive() && phase == Phase.SPIN) {
            drivetrain.driveRaw(0, 0, SPIN_POWER);

            LLResult result = robot.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.LLFiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    lockedTagId = fiducials.get(0).getFiducialId();
                    phase = Phase.CENTER;
                    timer.reset();
                    telemetry.addData("Locked tag", lockedTagId);
                }
            }

            if (timer.milliseconds() > SPIN_TIMEOUT_MS) {
                telemetry.addData("SPIN", "Timed out — no tag found");
                phase = Phase.DONE;
            }

            telemetry.addData("Phase", "SPIN");
            telemetry.update();
        }

        // Stop motors between phases
        drivetrain.driveRaw(0, 0, 0);

        // CENTER and APPROACH phases to be added in Task 3
        robot.limelight.stop();
    }
}
```

- [ ] **Step 2: Build and verify compilation**

```bash
./gradlew :TeamCode:build
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/AprilTagSeekAuto.java
git commit -m "feat: add AprilTagSeekAuto with SPIN phase"
```

---

### Task 3: Implement CENTER and APPROACH phases

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/AprilTagSeekAuto.java`

**Interfaces:**
- Consumes: `lockedTagId` (int) set by SPIN phase
- Consumes: `LLFiducialResult.getTargetXDegrees()` — bearing in degrees (negative = tag is left of center, positive = right)
- Consumes: `LLFiducialResult.getRobotPoseTargetSpace()` — 6-element double array `[x, y, z, roll, pitch, yaw]` in meters; `z` (index 2) is the forward distance to the tag

- [ ] **Step 1: Replace the CENTER/APPROACH stub in runOpMode()**

Replace the section after the SPIN phase stop (from `// CENTER and APPROACH phases` comment to `robot.limelight.stop()`) with:

```java
        // --- CENTER phase ---
        timer.reset();
        while (opModeIsActive() && phase == Phase.CENTER) {
            LLResult result = robot.limelight.getLatestResult();
            LLResultTypes.LLFiducialResult locked = findTag(result, lockedTagId);

            if (locked != null) {
                double bearing = locked.getTargetXDegrees();
                if (Math.abs(bearing) <= BEARING_THRESHOLD) {
                    drivetrain.driveRaw(0, 0, 0);
                    phase = Phase.APPROACH;
                    timer.reset();
                } else {
                    // keep spinning in the same direction, proportional nudge
                    double turnPower = Math.signum(bearing) * SPIN_POWER;
                    drivetrain.driveRaw(0, 0, turnPower);
                }
                telemetry.addData("CENTER bearing", "%.1f deg", bearing);
            } else {
                drivetrain.driveRaw(0, 0, SPIN_POWER); // keep spinning, tag temporarily lost
            }

            if (timer.milliseconds() > CENTER_TIMEOUT_MS) {
                telemetry.addData("CENTER", "Timed out — tag lost");
                phase = Phase.DONE;
            }

            telemetry.addData("Phase", "CENTER");
            telemetry.update();
        }

        drivetrain.driveRaw(0, 0, 0);

        // --- APPROACH phase ---
        while (opModeIsActive() && phase == Phase.APPROACH) {
            LLResult result = robot.limelight.getLatestResult();
            LLResultTypes.LLFiducialResult locked = findTag(result, lockedTagId);

            if (locked != null) {
                double bearing = locked.getTargetXDegrees();
                double[] pose  = locked.getRobotPoseTargetSpace();
                double rangeIn = Math.abs(pose[2]) * 39.37; // meters → inches

                if (rangeIn <= DESIRED_DISTANCE) {
                    drivetrain.driveRaw(0, 0, 0);
                    phase = Phase.DONE;
                    telemetry.addData("APPROACH", "Reached target at %.1f in", rangeIn);
                } else {
                    double drive = Math.min((rangeIn - DESIRED_DISTANCE) * SPEED_GAIN, MAX_DRIVE_SPEED);
                    double turn  = Math.max(-MAX_TURN_SPEED, Math.min(MAX_TURN_SPEED, bearing * TURN_GAIN));
                    drivetrain.driveRaw(drive, 0, turn);
                    telemetry.addData("APPROACH range", "%.1f in", rangeIn);
                    telemetry.addData("APPROACH bearing", "%.1f deg", bearing);
                }
            } else {
                drivetrain.driveRaw(0, 0, 0); // stop if tag lost
                phase = Phase.DONE;
                telemetry.addData("APPROACH", "Tag lost — stopping");
            }

            telemetry.addData("Phase", "APPROACH");
            telemetry.update();
        }

        drivetrain.driveRaw(0, 0, 0);
        robot.limelight.stop();
    }

    private LLResultTypes.LLFiducialResult findTag(LLResult result, int tagId) {
        if (result == null || !result.isValid()) return null;
        for (LLResultTypes.LLFiducialResult f : result.getFiducialResults()) {
            if (f.getFiducialId() == tagId) return f;
        }
        return null;
    }
```

**Important:** The closing brace of `runOpMode()` is now inside the replacement block above. The class still needs its own closing brace `}` at the end of the file. The full file structure is:

```
class AprilTagSeekAuto {
    constants...
    enum Phase...
    fields...

    runOpMode() {
        init
        waitForStart
        SPIN while loop
        CENTER while loop
        APPROACH while loop
        limelight.stop()
    }  // end runOpMode

    findTag() { ... }

}  // end class
```

- [ ] **Step 2: Build and verify compilation**

```bash
./gradlew :TeamCode:build
```

Expected: `BUILD SUCCESSFUL`. The most common error here is a mismatched brace — count them if the build fails.

- [ ] **Step 3: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/AprilTagSeekAuto.java
git commit -m "feat: implement CENTER and APPROACH phases in AprilTagSeekAuto"
```

---

### Task 4: On-Robot Verification

**Files:** None — deploy and observe.

**Before deploying, confirm in the Limelight web UI (http://limelight.local:5801):**
- Pipeline 0 is type "Fiducial Marker" (AprilTag)
- The correct AprilTag family is selected (36h11 is FTC standard)
- The camera exposure is set low (~6ms) to reduce motion blur during spin

- [ ] **Step 1: Deploy to robot**

Connect to the robot's Wi-Fi. Open the project in Android Studio and press **Run** (or use `./gradlew installDebug` if you have ADB set up). The OpMode `"AprilTag Seek"` will appear under Autonomous on the Driver Station.

- [ ] **Step 2: Test SPIN phase**

Place one AprilTag in front of the robot (within ~3 feet). Start the OpMode. The robot should:
- Begin spinning
- Stop spinning within ~1 second of the tag entering the camera FOV
- Telemetry should show `"Locked tag: <id>"`

If the robot spins the full timeout without locking: check the Limelight pipeline is running (green LED on Limelight) and the tag is in the 36h11 family.

- [ ] **Step 3: Test CENTER phase**

With the tag visible but off-center, the robot should continue to rotate slowly until the tag is within 5° of center, then stop. Telemetry shows `"CENTER bearing: X.X deg"`.

If the robot overshoots: reduce `SPIN_POWER`. If it never converges: tighten `BEARING_THRESHOLD` from 5° to 3°.

- [ ] **Step 4: Test APPROACH phase**

The robot should drive toward the tag and stop at ~12 inches. Telemetry shows `"APPROACH range: XX.X in"`.

If the range reading seems wrong: verify the Limelight has the correct tag size entered in the pipeline config (tag size in mm). This is required for accurate pose estimation.

If the robot drives past the tag: increase `DESIRED_DISTANCE`. If it stops too far: decrease it.

- [ ] **Step 5: End-to-end test with two tags**

Place two AprilTags around the robot (e.g. at 10 o'clock and 2 o'clock positions). The robot should lock the first one it sees during the spin and ignore the second. Confirm by reading the telemetry `"Locked tag: <id>"` and checking that it matches the tag that was first swept past.

- [ ] **Step 6: Commit any tuning changes to constants**

After tuning, commit the updated constants:
```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/AprilTagSeekAuto.java
git commit -m "tune: adjust AprilTagSeekAuto constants after on-robot testing"
```
