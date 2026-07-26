# Test-Vision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Provide a safe TeleOp named `Test-Vision` that shows every available result from the Limelight 3A's active detector pipeline.

**Architecture:** Simplify the existing `LimelightTest` OpMode into a read-only Limelight telemetry viewer. It reuses `RobotHardware` for hardware lookup, polls `getLatestResult()` once per control loop, and renders status, result metadata, and `DetectorResult` entries. It does not change the selected pipeline, so Driver Station telemetry reflects the pipeline already chosen in Limelight.

**Tech Stack:** FTC SDK 11.1.0; Limelight 3A `Limelight3A`, `LLStatus`, `LLResult`, `LLResultTypes.DetectorResult`; Java Android OpMode.

## Global Constraints

- Keep `LimelightTest` annotated `@TeleOp(name = "Test-Vision", group = "Test")`.
- Reuse `RobotHardware.init(hardwareMap)` and `robot.limelight`; do not create a second hardware lookup or wrapper.
- Use only installed SDK calls: `isConnected()`, `getStatus()`, `getLatestResult()`, `getPipelineIndex()`, `getCaptureLatency()`, `getTargetingLatency()`, `getDetectorResults()`, `getBotpose()`, and `DetectorResult` label/confidence/angle/area accessors.
- `DetectorResult` has no pose accessor in FTC SDK 11.1.0; show result-level `getBotpose()` as the available robot-space pose.
- Do not delete any files: every TeamCode vision source is referenced by a production OpMode or a unit test.

---

### Task 1: Replace the drive/AprilTag test with a general Limelight results viewer

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/LimelightTest.java:1-225`
- Test: TeamCode compile target `:TeamCode:compileDebugJavaWithJavac`

**Interfaces:**
- Consumes: `RobotHardware.init(HardwareMap)`, `Limelight3A.start()`, `Limelight3A.getLatestResult()`, `Limelight3A.getStatus()`.
- Produces: a Driver Station entry named `Test-Vision` that reports connection, current pipeline, valid-frame state, capture/targeting/total latency, detector count, target data, and available robot pose.

- [ ] **Step 1: Establish the pre-change compile baseline**

Run: `./gradlew :TeamCode:compileDebugJavaWithJavac`

Expected: PASS. This confirms the checked-in FTC/Limelight SDK classes are resolvable before replacing the existing test OpMode.

- [ ] **Step 2: Replace the test OpMode implementation**

Remove the scan/turn/drive state machine, drivetrain dependency, forced AprilTag pipeline switch, and AprilTag helpers. Retain the `RobotHardware` initialization and Limelight lifecycle. Use this control-loop outline:

```java
LLResult result = null;
LLStatus status = null;
String communicationError = null;
try {
    status = robot.limelight.getStatus();
    result = robot.limelight.getLatestResult();
} catch (RuntimeException e) {
    communicationError = e.getClass().getSimpleName();
}

telemetry.addLine("=== LIMELIGHT STATUS ===");
telemetry.addData("Connected", robot.limelight.isConnected() ? "Yes" : "No");
if (status != null) {
    telemetry.addData("Pipeline", "%d (%s)", status.getPipelineIndex(), status.getPipelineType());
} else {
    telemetry.addData("Pipeline", "Unavailable");
}
```

For a non-null valid result, display `result.getPipelineIndex()`, `getCaptureLatency()`, `getTargetingLatency()`, their sum, and `result.getBotpose()` when non-null. Obtain `List<LLResultTypes.DetectorResult>` through `getDetectorResults()`, treating null as empty. Report `No targets detected.` for an empty list; otherwise enumerate each target’s `getClassName()`, `getConfidence()`, `getTargetXDegrees()`, `getTargetYDegrees()`, and `getTargetArea()` under a `=== DETECTED TARGETS ===` header. For null, invalid, disconnected, or exception cases, render a clear result state and safely continue. Add a short comment that `getLatestResult()` is intentionally called every loop and `DetectorResult` exposes no pose in this SDK.

- [ ] **Step 3: Compile against the installed API**

Run: `./gradlew :TeamCode:compileDebugJavaWithJavac`

Expected: PASS. This is the API compatibility test: it proves each Limelight call used by the OpMode exists in the SDK version installed by this project.

- [ ] **Step 4: Inspect the final diff and source references**

Run: `git diff --check && rg -n "LimelightVision|BallDetection|BallGroup|BallTarget|BallColor|BallGrouping|BallVisionConfig|TargetPersistence" TeamCode/src/main/java TeamCode/src/test/java -g '*.java'`

Expected: no whitespace errors; each TeamCode vision type continues to have a production or test reference. Leave all vision source files intact.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/LimelightTest.java
git commit -m "feat: add Limelight results telemetry test"
```
