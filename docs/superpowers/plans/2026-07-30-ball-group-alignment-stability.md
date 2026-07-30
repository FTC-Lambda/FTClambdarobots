# Ball-Group Alignment Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make ball-group alignment choose the most-ball/closest stable group, turn promptly, settle without hunting, and expose safe runtime tuning on Driver Station.

**Architecture:** Keep perception and hardware responsibilities where they are, but add a pure-Java control layer between `BallTarget` and `Drivetrain`. `BallGrouping` ranks groups, `TargetPersistence` owns lock/challenger/filter state, `BallAlignmentController` owns turn-control state, and both alignment OpModes consume those shared components.

**Tech Stack:** Java, FTC SDK `LinearOpMode`, Limelight 3A SDK, JUnit 4, Gradle Android module `TeamCode`.

## Global Constraints

- Follow the approved design in `docs/superpowers/specs/2026-07-29-ball-group-alignment-stability-design.md`.
- Do not change existing `VISION_SEEK_*` constants or AprilTag controller construction.
- Ball acquisition priority is ball count, apparent closeness, confidence, then absolute weighted horizontal error.
- A challenger needs three consecutive fresh confirmations before switching the lock.
- Alignment begins beyond 4 degrees, stops at or below 2 degrees, and never exceeds 0.65 turn power.
- A held frame cannot authorize a nonzero command beyond 100 ms from the last fresh target.
- Target loss, manual mode, configuration changes, and OpMode shutdown command zero immediately.
- Runtime tuning is session-only and never writes files, preferences, or global constants.
- Keep the control loop non-blocking and route all motor power through `Drivetrain`.

## File Map

- Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/BallAlignmentConfig.java`: immutable validated alignment settings.
- Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/BallAlignmentController.java`: FTC-free stateful turn controller and diagnostic result.
- Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/BallAlignmentTuningModel.java`: parameter selection and fine/coarse runtime edits.
- Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/BallTargetingConfig.java`: immutable validated arbitration/filter settings.
- Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/BallGroupAlignmentTuner.java`: Driver Station tuning OpMode.
- Create corresponding JUnit tests under `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/control/`.
- Modify `Constants`, `BallVisionConfig`, `BallGrouping`, `TargetPersistence`, `LimelightVision`, and `BallGroupingAligningTest`.
- Extend existing `BallGroupingTest` and `TargetPersistenceTest`.
- Modify `docs/OPMODES.md` to document the two ball-alignment Test OpModes and tuner controls.

---

### Task 1: Validated Runtime Configuration Objects

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Constants.java:35`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/BallVisionConfig.java:75`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/BallAlignmentConfig.java`
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/BallTargetingConfig.java`
- Create: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/control/BallAlignmentConfigTest.java`
- Create: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/BallTargetingConfigTest.java`

**Interfaces:**
- Produces: `BallAlignmentConfig.defaults()`, getters, and `withTurnKp`, `withTurnKi`, `withTurnKd`, `withDerivativeFilter`, `withStartCorrectionDeg`, `withStopCorrectionDeg`, `withMaxTurnPower`, `withStaticFrictionPower`, `withTurnSlewRate`, `withCommandHoldMs`.
- Produces: `BallTargetingConfig.defaults()`, `getSwitchConfirmFrames()`, `getAimFilterGain()`, `withSwitchConfirmFrames(int)`, and `withAimFilterGain(double)`.
- Validation: all values finite; nonnegative gains/rates; `0 < stop <= start`; `0 < max <= 1`; `0 <= friction <= max`; hold time positive; confirmation frames `[1,10]`; filter gain `(0,1]`.

- [ ] **Step 1: Write failing configuration tests**

```java
@Test
public void alignmentConfigClampsDependentBounds() {
    BallAlignmentConfig c = BallAlignmentConfig.defaults()
            .withMaxTurnPower(0.10)
            .withStaticFrictionPower(0.40)
            .withStartCorrectionDeg(1.0)
            .withStopCorrectionDeg(3.0);
    assertEquals(0.10, c.getMaxTurnPower(), 1e-9);
    assertEquals(0.10, c.getStaticFrictionPower(), 1e-9);
    assertTrue(c.getStopCorrectionDeg() <= c.getStartCorrectionDeg());
}

@Test
public void nonFiniteAlignmentValuesReturnSafeBounds() {
    BallAlignmentConfig c = BallAlignmentConfig.defaults()
            .withTurnKp(Double.NaN)
            .withTurnSlewRate(Double.POSITIVE_INFINITY);
    assertTrue(Double.isFinite(c.getTurnKp()));
    assertTrue(Double.isFinite(c.getTurnSlewRate()));
    assertTrue(c.getTurnKp() >= 0.0);
    assertTrue(c.getTurnSlewRate() >= 0.0);
}

@Test
public void targetingConfigClampsFramesAndFilter() {
    BallTargetingConfig low = BallTargetingConfig.defaults()
            .withSwitchConfirmFrames(0)
            .withAimFilterGain(-1.0);
    BallTargetingConfig high = BallTargetingConfig.defaults()
            .withSwitchConfirmFrames(50)
            .withAimFilterGain(5.0);
    assertEquals(1, low.getSwitchConfirmFrames());
    assertTrue(low.getAimFilterGain() > 0.0);
    assertEquals(10, high.getSwitchConfirmFrames());
    assertEquals(1.0, high.getAimFilterGain(), 1e-9);
}
```

- [ ] **Step 2: Run the focused tests and verify missing-type failures**

Run:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.control.BallAlignmentConfigTest" --tests "org.firstinspires.ftc.teamcode.vision.BallTargetingConfigTest"
```

Expected: FAIL because both configuration classes are absent.

- [ ] **Step 3: Add the exact default constants**

Add to `Constants`:

```java
public static final double BALL_ALIGN_TURN_KP = 0.040;
public static final double BALL_ALIGN_TURN_KI = 0.0;
public static final double BALL_ALIGN_TURN_KD = 0.003;
public static final double BALL_ALIGN_TURN_DERIVATIVE_FILTER = 0.20;
public static final double BALL_ALIGN_START_CORRECTION_DEG = 4.0;
public static final double BALL_ALIGN_STOP_CORRECTION_DEG = 2.0;
public static final double BALL_ALIGN_MAX_TURN_POWER = 0.65;
public static final double BALL_ALIGN_STATIC_FRICTION_POWER = 0.12;
public static final double BALL_ALIGN_TURN_SLEW_RATE = 6.0;
public static final long BALL_ALIGN_COMMAND_HOLD_MS = 100L;
```

Add to `BallVisionConfig`:

```java
public static final int BALL_TARGET_SWITCH_CONFIRM_FRAMES = 3;
public static final double BALL_TARGET_AIM_FILTER_GAIN = 0.50;
```

- [ ] **Step 4: Implement immutable validated configurations**

Use constructor validation and return new objects from every `with...` method. `defaults()` must copy the constants above:

```java
public static BallAlignmentConfig defaults() {
    return new BallAlignmentConfig(
            Constants.BALL_ALIGN_TURN_KP,
            Constants.BALL_ALIGN_TURN_KI,
            Constants.BALL_ALIGN_TURN_KD,
            Constants.BALL_ALIGN_TURN_DERIVATIVE_FILTER,
            Constants.BALL_ALIGN_START_CORRECTION_DEG,
            Constants.BALL_ALIGN_STOP_CORRECTION_DEG,
            Constants.BALL_ALIGN_MAX_TURN_POWER,
            Constants.BALL_ALIGN_STATIC_FRICTION_POWER,
            Constants.BALL_ALIGN_TURN_SLEW_RATE,
            Constants.BALL_ALIGN_COMMAND_HOLD_MS);
}

private static double finiteOr(double value, double fallback) {
    return Double.isFinite(value) ? value : fallback;
}
```

For deadbands, clamp stop after start: `start = max(0.25, start)` and `stop = clamp(stop, 0.25, start)`. For non-finite wither inputs, retain the current field value. Apply the same immutable pattern in `BallTargetingConfig`.

- [ ] **Step 5: Run configuration tests**

Run the command from Step 2.

Expected: PASS.

- [ ] **Step 6: Commit**

```powershell
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Constants.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/BallVisionConfig.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/BallAlignmentConfig.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/BallTargetingConfig.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/control/BallAlignmentConfigTest.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/BallTargetingConfigTest.java
git commit -m "feat: add ball alignment runtime configuration"
```

---

### Task 2: Deterministic Multi-Group Priority

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/BallGrouping.java:284`
- Modify: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/BallGroupingTest.java:181`

**Interfaces:**
- Consumes: existing `BallGroup` metrics and optional `BallColor`.
- Produces: `isHigherPriority(BallGroup candidate, BallGroup incumbent)` and `selectPriorityGroup(List<BallGroup>, BallColor)`.
- Preserves: `selectClosestGroup(...)` for callers that explicitly request apparent closeness.
- Changes: `selectBestGroup(...)` delegates to `selectPriorityGroup(...)`.

- [ ] **Step 1: Add failing priority-order tests**

```java
@Test
public void moreBallsBeatCloserSmallerGroup() {
    BallGroup threeFar = BallGrouping.buildGroup(Arrays.asList(
            det("green", 0.8, 12, 0, 0.01, 0, 0, 0, 0, false),
            det("green", 0.8, 14, 0, 0.01, 0, 0, 0, 0, false),
            det("green", 0.8, 16, 0, 0.01, 0, 0, 0, 0, false)), null);
    BallGroup twoNear = BallGrouping.buildGroup(Arrays.asList(
            det("green", 0.8, -10, 0, 0.05, 0, 0, 0, 0, false),
            det("green", 0.8, -8, 0, 0.05, 0, 0, 0, 0, false)), null);
    assertEquals(threeFar,
            BallGrouping.selectPriorityGroup(Arrays.asList(twoNear, threeFar), null).get());
}

@Test
public void equalCountUsesClosenessThenConfidenceThenCenter() {
    BallGroup farther = BallGrouping.buildGroup(
            Collections.singletonList(det("green", 0.99, 0, 0, 0.02, 0, 0, 0, 0, false)), null);
    BallGroup nearer = BallGrouping.buildGroup(
            Collections.singletonList(det("green", 0.60, 15, 0, 0.03, 0, 0, 0, 0, false)), null);
    assertTrue(BallGrouping.isHigherPriority(nearer, farther));
}
```

- [ ] **Step 2: Run and verify failure**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.vision.BallGroupingTest"
```

Expected: FAIL because the priority APIs do not exist.

- [ ] **Step 3: Implement the lexicographic comparator and selection**

```java
public static boolean isHigherPriority(BallGroup candidate, BallGroup incumbent) {
    if (candidate == null) return false;
    if (incumbent == null) return true;
    if (candidate.getSize() != incumbent.getSize()) {
        return candidate.getSize() > incumbent.getSize();
    }
    int areaCmp = Double.compare(apparentCloseness(candidate), apparentCloseness(incumbent));
    if (areaCmp != 0) return areaCmp > 0;
    int confCmp = Double.compare(candidate.getAverageConfidence(), incumbent.getAverageConfidence());
    if (confCmp != 0) return confCmp > 0;
    return Math.abs(candidate.getWeightedTxDeg()) < Math.abs(incumbent.getWeightedTxDeg());
}
```

Factor the existing desired-color fallback into a private `eligibleGroups(...)` helper and iterate those groups with `isHigherPriority`. Keep the legacy closest selector intact and update both `selectBestGroup` overloads to priority selection.

- [ ] **Step 4: Run grouping tests**

Run the command from Step 2.

Expected: PASS, including existing filtering/group-span tests.

- [ ] **Step 5: Commit**

```powershell
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/BallGrouping.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/BallGroupingTest.java
git commit -m "feat: prioritize ball groups by count and closeness"
```

---

### Task 3: Stable Target Lock, Challenger Confirmation, and Aim Filtering

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/TargetPersistence.java:11`
- Modify: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/TargetPersistenceTest.java:13`

**Interfaces:**
- Consumes: `BallTargetingConfig` and `BallGrouping.isHigherPriority`.
- Produces: constructors `TargetPersistence()` and `TargetPersistence(BallTargetingConfig)`.
- Produces: `setConfig(BallTargetingConfig)`, `getPendingGroup()`, `getPendingConfirmationFrames()`, `getRawLockedTxDeg()`, and existing `getStickyGroup()`.
- Behavior: associated current observation wins identity continuity; challenger switches only after configured consecutive frames; published target bearing is the filtered weighted bearing.
- Behavior: changing desired color clears locked/pending state so the next fresh frame reacquires under the new eligibility filter.

- [ ] **Step 1: Replace immediate-switch expectations with failing persistence scenarios**

Add test helpers that build distinct group members rather than `Collections.nCopies` when weighted centers matter. Add these tests:

```java
@Test
public void challengerRequiresThreeConsecutiveFreshFrames() {
    TargetPersistence p = new TargetPersistence(
            BallTargetingConfig.defaults().withSwitchConfirmFrames(3));
    BallGroup locked = group(15, 0, 1, 0.02);
    BallGroup challenger = group(-10, 0, 2, 0.03);
    assertEquals(15.0, p.update(1000, 1.0, Arrays.asList(locked)).getHorizontalErrorDeg(), 1e-6);
    assertEquals(15.0, p.update(1050, 2.0, Arrays.asList(locked, challenger)).getHorizontalErrorDeg(), 1e-6);
    assertEquals(1, p.getPendingConfirmationFrames());
    assertEquals(15.0, p.update(1100, 3.0, Arrays.asList(locked, challenger)).getHorizontalErrorDeg(), 1e-6);
    BallTarget switched = p.update(1150, 4.0, Arrays.asList(locked, challenger));
    assertEquals(-10.0, switched.getHorizontalErrorDeg(), 1e-6);
    assertEquals(0, p.getPendingConfirmationFrames());
}

@Test
public void oneFrameCountJumpDoesNotSwitchLock() {
    TargetPersistence p = new TargetPersistence();
    BallGroup locked = group(0, 0, 2, 0.02);
    p.update(1000, 1.0, Arrays.asList(locked));
    BallGroup transientThree = group(18, 0, 3, 0.03);
    p.update(1050, 2.0, Arrays.asList(locked, transientThree));
    BallTarget recovered = p.update(1100, 3.0, Arrays.asList(locked));
    assertTrue(Math.abs(recovered.getHorizontalErrorDeg()) < 1.0);
    assertEquals(0, p.getPendingConfirmationFrames());
}

@Test
public void aimpointUsesConfiguredExponentialFilter() {
    TargetPersistence p = new TargetPersistence(
            BallTargetingConfig.defaults().withAimFilterGain(0.50));
    p.update(1000, 1.0, Arrays.asList(group(0, 0, 2)));
    BallTarget next = p.update(1050, 2.0, Arrays.asList(group(4, 0, 2)));
    assertEquals(2.0, next.getHorizontalErrorDeg(), 1e-6);
    assertEquals(4.0, p.getRawLockedTxDeg(), 1e-6);
}

@Test
public void missingLockHoldsOldTargetWhileCandidateConfirms() {
    TargetPersistence p = new TargetPersistence();
    p.update(1000, 1.0, Arrays.asList(group(20, 0, 2)));
    BallTarget pending = p.update(1050, 2.0, Arrays.asList(group(-15, 0, 3)));
    assertTrue(pending.isValid());
    assertFalse(pending.isFresh());
    assertEquals(50L, pending.getAgeMs());
}

@Test
public void changingDesiredColorClearsLockAndPendingChallenger() {
    TargetPersistence p = new TargetPersistence();
    p.update(1000, 1.0, Arrays.asList(group(0, 0, 2)));
    p.update(1050, 2.0, Arrays.asList(group(0, 0, 2), group(18, 0, 3)));
    p.setDesiredColor(BallColor.PURPLE);
    assertFalse(p.getStickyGroup().isPresent());
    assertFalse(p.getPendingGroup().isPresent());
    assertEquals(0, p.getPendingConfirmationFrames());
}
```

Also update the old `switchesWhenAnotherGroupIsClearlyCloser` test to provide three confirming fresh frames.

- [ ] **Step 2: Run persistence tests and verify failure**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.vision.TargetPersistenceTest"
```

Expected: FAIL because switching is immediate and filtering/pending diagnostics do not exist.

- [ ] **Step 3: Implement explicit lock and challenger state**

Add fields:

```java
private BallTargetingConfig config;
private BallGroup stickyGroup;
private BallGroup pendingGroup;
private int pendingConfirmationFrames;
private double filteredTxDeg;
private double filteredTyDeg;
private double rawLockedTxDeg;
private double rawLockedTyDeg;
private boolean hasFilteredAim;
```

On each fresh frame:

1. Select the current observation as the in-gate group with minimum angular distance to the previous sticky group.
2. Select the best remaining group using priority ordering.
3. If the current observation exists, update `stickyGroup`, `lastSeenHubMs`, and the filtered aim.
4. A challenger with greater ball count outranks immediately for confirmation purposes. With equal count, require both higher priority and `challengerArea > currentArea * CLOSEST_AREA_SWITCH_RATIO`.
5. Increment pending count only when the new challenger associates with `pendingGroup`; otherwise restart at one.
6. Promote at `config.getSwitchConfirmFrames()`, reseed the filter from raw weighted coordinates, and clear pending state.
7. If current observation is missing, return the old target as held while the best candidate follows the same confirmation path.

Use this filter helper only for the current lock:

```java
private void updateFilteredAim(BallGroup observation, boolean reseed) {
    rawLockedTxDeg = observation.getWeightedTxDeg();
    rawLockedTyDeg = observation.getWeightedTyDeg();
    if (reseed || !hasFilteredAim) {
        filteredTxDeg = rawLockedTxDeg;
        filteredTyDeg = rawLockedTyDeg;
        hasFilteredAim = true;
        return;
    }
    double a = config.getAimFilterGain();
    filteredTxDeg += a * (rawLockedTxDeg - filteredTxDeg);
    filteredTyDeg += a * (rawLockedTyDeg - filteredTyDeg);
}
```

`toTarget(...)` must publish `filteredTxDeg`/`filteredTyDeg`, not the raw group angles. `setConfig(...)` validates by construction and calls `clear()` so an old lock is never filtered with new semantics.

Update `setDesiredColor(...)` to call `clear()` only when the requested color actually changes, then store the new color. Preserve the new desired color across `clear()` by keeping `clear()` limited to lock/filter/timing state.

- [ ] **Step 4: Run grouping and persistence tests**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.vision.BallGroupingTest" --tests "org.firstinspires.ftc.teamcode.vision.TargetPersistenceTest"
```

Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/TargetPersistence.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/TargetPersistenceTest.java
git commit -m "feat: confirm ball target switches across frames"
```

---

### Task 4: FTC-Free Ball Alignment Controller

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/BallAlignmentController.java`
- Create: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/control/BallAlignmentControllerTest.java`

**Interfaces:**
- Consumes: `BallAlignmentConfig`, `BallTarget`, `PIDController`, and `SlewRateLimiter`.
- Produces: `BallAlignmentController(BallAlignmentConfig)`, `setConfig(BallAlignmentConfig)`, `update(BallTarget,long,double)`, and `reset()`.
- Produces nested enum `Action { NO_TARGET, CENTERED, CORRECTING, HOLDING_COMMAND, COMMAND_EXPIRED, REVERSAL_GUARD }`.
- Produces immutable nested `Result` getters for bearing, P/I/D, raw PID, requested turn, applied turn, correction-active, static-friction-applied, slew-limited, and `Action`.

- [ ] **Step 1: Write controller behavior tests**

Use:

```java
private static BallTarget target(double bearing, boolean fresh, long ageMs) {
    return new BallTarget(true, fresh, bearing, 0.0, 0.8, 3, ageMs,
            0.0, 3, 0, null);
}
```

Add separate tests for:

```java
@Test
public void hysteresisStopsAtTwoAndRestartsAboveFour() {
    BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
    assertTrue(c.update(target(8.0, true, 0), 1000, 0.02).isCorrectionActive());
    assertEquals(0.0, c.update(target(2.0, true, 0), 1050, 0.02).getAppliedTurn(), 1e-9);
    assertFalse(c.update(target(3.5, true, 0), 1100, 0.02).isCorrectionActive());
    assertTrue(c.update(target(4.1, true, 0), 1150, 0.02).isCorrectionActive());
}

@Test
public void commandExpiresAfterOneHundredMilliseconds() {
    BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
    c.update(target(12.0, true, 0), 1000, 0.02);
    BallAlignmentController.Result held = c.update(target(12.0, false, 80), 1080, 0.02);
    BallAlignmentController.Result expired = c.update(target(12.0, false, 101), 1101, 0.02);
    assertEquals(BallAlignmentController.Action.HOLDING_COMMAND, held.getAction());
    assertEquals(BallAlignmentController.Action.COMMAND_EXPIRED, expired.getAction());
    assertTrue(Math.abs(expired.getAppliedTurn()) <= Math.abs(held.getAppliedTurn()));
}

@Test
public void invalidTargetStopsImmediatelyAndResetClearsState() {
    BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
    c.update(target(12.0, true, 0), 1000, 0.02);
    assertEquals(0.0, c.update(BallTarget.none(), 1020, 0.02).getAppliedTurn(), 1e-9);
    c.reset();
    assertEquals(0.0, c.update(BallTarget.none(), 1040, 0.02).getAppliedTurn(), 1e-9);
}

@Test
public void oppositeFreshCommandStopsBeforeReversing() {
    BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
    assertTrue(c.update(target(12.0, true, 0), 1000, 0.02).getAppliedTurn() > 0);
    BallAlignmentController.Result guarded = c.update(target(-12.0, true, 0), 1050, 0.02);
    assertEquals(BallAlignmentController.Action.REVERSAL_GUARD, guarded.getAction());
    assertEquals(0.0, guarded.getAppliedTurn(), 1e-9);
    assertTrue(c.update(target(-12.0, true, 0), 1100, 0.02).getAppliedTurn() < 0);
}
```

Add assertions that output is bounded by 0.65, static friction is absent while centered, repeated frames do not change PID terms, invalid/non-finite time inputs stop safely, and `setConfig(...)` resets output.

- [ ] **Step 2: Run the focused controller test**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.control.BallAlignmentControllerTest"
```

Expected: FAIL because the controller is absent.

- [ ] **Step 3: Implement the state machine**

Construct a fresh `PIDController` and `SlewRateLimiter` whenever configuration changes. Seed the new limiter with `calculate(0.0, 0.0)`. Keep `lastFreshMs`, `requestedTurn`, `appliedTurn`, `correctionActive`, and `pendingReverseSign`.

Fresh-frame core:

```java
if (!correctionActive && Math.abs(bearing) > config.getStartCorrectionDeg()) {
    correctionActive = true;
}
if (correctionActive && Math.abs(bearing) <= config.getStopCorrectionDeg()) {
    return stopAndResetControl(Action.CENTERED, bearing);
}
double measurementDt = lastFreshMs == Long.MIN_VALUE
        ? safeLoopDt(loopDtSeconds)
        : Math.max(1e-4, (nowMs - lastFreshMs) / 1000.0);
double rawPid = pid.calculate(bearing, 0.0, measurementDt);
double request = clamp(rawPid, -config.getMaxTurnPower(), config.getMaxTurnPower());
if (request != 0.0 && Math.abs(request) < config.getStaticFrictionPower()) {
    request = Math.copySign(config.getStaticFrictionPower(), request);
}
```

Before applying a fresh request, compare its sign with nonzero `appliedTurn`. The first opposite sign sets `pendingReverseSign`, returns zero, resets/seeds the limiter, and reports `REVERSAL_GUARD`; a subsequent fresh frame with the same sign may proceed. Held frames retain the request only through `commandHoldMs`, then pass zero to the limiter. Invalid targets and invalid/nonmonotonic time call the immediate-zero reset path.

- [ ] **Step 4: Run controller and utility tests**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.control.BallAlignmentControllerTest"
```

Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/BallAlignmentController.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/control/BallAlignmentControllerTest.java
git commit -m "feat: add stable ball alignment controller"
```

---

### Task 5: Wire Runtime Target Configuration and Diagnostics Through Vision

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/LimelightVision.java:22`
- Create: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/LimelightVisionApiTest.java`

**Interfaces:**
- Produces: `setBallTargetingConfig(BallTargetingConfig)`, `getBallTargetingConfig()`, `getPendingBallGroup()`, `getPendingBallConfirmationFrames()`, and `getRawLockedTxDeg()`.
- Preserves: pipeline lifecycle, detection parsing, and `getTarget()`.

- [ ] **Step 1: Add a reflection-based API test**

This verifies the FTC-facing class exposes the planned methods without constructing hardware:

```java
@Test
public void exposesTargetingConfigurationAndDiagnostics() throws Exception {
    assertEquals(void.class, LimelightVision.class
            .getMethod("setBallTargetingConfig", BallTargetingConfig.class).getReturnType());
    assertEquals(BallTargetingConfig.class, LimelightVision.class
            .getMethod("getBallTargetingConfig").getReturnType());
    assertEquals(Optional.class, LimelightVision.class
            .getMethod("getPendingBallGroup").getReturnType());
    assertEquals(int.class, LimelightVision.class
            .getMethod("getPendingBallConfirmationFrames").getReturnType());
    assertEquals(double.class, LimelightVision.class
            .getMethod("getRawLockedTxDeg").getReturnType());
}
```

- [ ] **Step 2: Run and verify failure**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.vision.LimelightVisionApiTest"
```

Expected: FAIL because the methods do not exist.

- [ ] **Step 3: Delegate configuration and diagnostics**

Store the active targeting config in `LimelightVision`, initialize `TargetPersistence` with it, and delegate:

```java
public void setBallTargetingConfig(BallTargetingConfig config) {
    targetingConfig = config == null ? BallTargetingConfig.defaults() : config;
    persistence.setConfig(targetingConfig);
    target = BallTarget.none();
}

public Optional<BallGroup> getPendingBallGroup() {
    return persistence.getPendingGroup();
}
```

Extend `addTelemetry` with raw/filtered bearing, locked/pending sizes, and confirmation `current/configured`. Keep all null/empty handling non-throwing.

- [ ] **Step 4: Run the API test and compile TeamCode**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.vision.LimelightVisionApiTest"
.\gradlew.bat :TeamCode:compileDebugJavaWithJavac
```

Expected: both commands PASS.

- [ ] **Step 5: Commit**

```powershell
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/LimelightVision.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/vision/LimelightVisionApiTest.java
git commit -m "feat: expose ball target lock diagnostics"
```

---

### Task 6: Refactor the Existing Alignment Test to the Shared Controller

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/BallGroupingAligningTest.java:39`
- Test: TeamCode Java compilation

**Interfaces:**
- Consumes: `BallAlignmentConfig.defaults()` and `BallAlignmentController`.
- Removes: local PID, `VisionDeadband`, `SlewRateLimiter`, `MAX_TURN_POWER`, `TURN_MIN_POWER`, `requestedTurn`, and `smoothedTurn`.
- Preserves: existing MANUAL/ALIGN and color controls.

- [ ] **Step 1: Compile the current baseline**

```powershell
.\gradlew.bat :TeamCode:compileDebugJavaWithJavac
```

Expected: PASS before the refactor.

- [ ] **Step 2: Replace local control composition**

Initialize:

```java
BallAlignmentController alignment = new BallAlignmentController(BallAlignmentConfig.defaults());
BallAlignmentController.Result alignmentResult =
        alignment.update(BallTarget.none(), System.currentTimeMillis(), 0.0);
```

On every mode/color transition call `alignment.reset()`. In ALIGN:

```java
alignmentResult = alignment.update(target, System.currentTimeMillis(), loopDt);
drivetrain.driveRaw(0.0, 0.0, alignmentResult.getAppliedTurn());
```

In MANUAL retain `drivetrain.drive(...)`; on target loss let the controller's invalid-target path command immediate zero. Remove the second bearing source from control. Display `target.getHorizontalErrorDeg()` as canonical bearing and `selectedGroup.getAverageTxDeg()` only as the diagnostic arithmetic mean.

- [ ] **Step 3: Add controller diagnostics to telemetry**

Report `Action`, correction active, P/I/D, raw PID, requested/applied power, friction applied, slew limited, raw/filtered vision bearing, locked/pending group sizes, and confirmation frames. Do not recompute any of those decisions in the OpMode.

- [ ] **Step 4: Compile and run all local unit tests**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest
.\gradlew.bat :TeamCode:compileDebugJavaWithJavac
```

Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/BallGroupingAligningTest.java
git commit -m "refactor: use shared ball alignment controller"
```

---

### Task 7: Pure-Java Driver Station Tuning Model

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/BallAlignmentTuningModel.java`
- Create: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/control/BallAlignmentTuningModelTest.java`

**Interfaces:**
- Consumes: `BallAlignmentConfig` and `BallTargetingConfig`.
- Produces enum `Parameter` for the 12 design parameters.
- Produces enum `ChangeDomain { ALIGNMENT, TARGETING }`.
- Produces: `selectPrevious()`, `selectNext()`, `adjust(int direction, boolean coarse)`, `getSelectedParameter()`, `getAlignmentConfig()`, `getTargetingConfig()`, `getSelectedValue()`, `getSelectedDefaultValue()`, `getFineIncrement()`, and `getCoarseIncrement()`.

- [ ] **Step 1: Write parameter isolation and wraparound tests**

```java
@Test
public void selectionWrapsAndAdjustmentChangesOnlySelectedField() {
    BallAlignmentTuningModel m = new BallAlignmentTuningModel();
    assertEquals(BallAlignmentTuningModel.Parameter.TURN_KP, m.getSelectedParameter());
    double kdBefore = m.getAlignmentConfig().getTurnKd();
    assertEquals(BallAlignmentTuningModel.ChangeDomain.ALIGNMENT, m.adjust(1, false));
    assertEquals(Constants.BALL_ALIGN_TURN_KP + 0.002,
            m.getAlignmentConfig().getTurnKp(), 1e-9);
    assertEquals(kdBefore, m.getAlignmentConfig().getTurnKd(), 1e-9);
    m.selectPrevious();
    assertEquals(BallAlignmentTuningModel.Parameter.AIM_FILTER_GAIN,
            m.getSelectedParameter());
}

@Test
public void coarseAdjustmentUsesDocumentedIncrement() {
    BallAlignmentTuningModel m = new BallAlignmentTuningModel();
    m.selectNext(); // TURN_KI
    m.selectNext(); // TURN_KD
    m.adjust(1, true);
    assertEquals(Constants.BALL_ALIGN_TURN_KD + 0.005,
            m.getAlignmentConfig().getTurnKd(), 1e-9);
}
```

Add a loop over all `Parameter` values asserting an adjustment changes only its declared domain and all resulting configs remain valid.

- [ ] **Step 2: Run and verify failure**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest --tests "org.firstinspires.ftc.teamcode.control.BallAlignmentTuningModelTest"
```

Expected: FAIL because the tuning model is absent.

- [ ] **Step 3: Implement the exact parameter table**

Encode the fine/coarse increments from the spec in the enum constructor:

```java
TURN_KP("turn kP", 0.002, 0.010, ChangeDomain.ALIGNMENT),
TURN_KI("turn kI", 0.0005, 0.0020, ChangeDomain.ALIGNMENT),
TURN_KD("turn kD", 0.001, 0.005, ChangeDomain.ALIGNMENT),
DERIVATIVE_FILTER("derivative filter", 0.05, 0.20, ChangeDomain.ALIGNMENT),
START_DEADBAND("start deadband", 0.25, 1.00, ChangeDomain.ALIGNMENT),
STOP_DEADBAND("stop deadband", 0.25, 1.00, ChangeDomain.ALIGNMENT),
MAX_TURN_POWER("max turn power", 0.05, 0.10, ChangeDomain.ALIGNMENT),
STATIC_FRICTION("static friction", 0.01, 0.05, ChangeDomain.ALIGNMENT),
TURN_SLEW_RATE("turn slew rate", 0.5, 2.0, ChangeDomain.ALIGNMENT),
COMMAND_HOLD_MS("command hold ms", 25, 100, ChangeDomain.ALIGNMENT),
SWITCH_CONFIRM_FRAMES("switch confirm frames", 1, 1, ChangeDomain.TARGETING),
AIM_FILTER_GAIN("aim filter gain", 0.05, 0.20, ChangeDomain.TARGETING);
```

Use a `switch` in `adjust(...)` that calls exactly one immutable config wither and returns the selected domain. `direction` is normalized to `-1`, `0`, or `1`.

- [ ] **Step 4: Run tuning-model tests**

Run the command from Step 2.

Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/control/BallAlignmentTuningModel.java TeamCode/src/test/java/org/firstinspires/ftc/teamcode/control/BallAlignmentTuningModelTest.java
git commit -m "feat: add ball alignment tuning model"
```

---

### Task 8: Driver Station Ball Alignment Tuner OpMode

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/BallGroupAlignmentTuner.java`
- Modify: `docs/OPMODES.md`
- Test: TeamCode Java compilation

**Interfaces:**
- Consumes: `BallAlignmentTuningModel`, `BallAlignmentController`, `LimelightVision`, `Drivetrain`.
- Driver Station name: `Ball-Group Alignment Tuner`, group `Test`.
- Controls: A align, B manual, X green, Y purple, either bumper any color, D-pad up/down select, D-pad left/right adjust, left-stick button selects coarse increment.

- [ ] **Step 1: Create the OpMode shell and edge-triggered input state**

Use the same hardware/vision lifecycle and `try/finally` stop path as `BallGroupingAligningTest`. Track previous booleans for A/B/X/Y, bumpers, and four D-pad directions. Never apply repeated held-button changes.

Core adjustment:

```java
boolean coarse = gamepad1.left_stick_button;
if (gamepad1.dpad_up && !prevDpadUp) tuning.selectPrevious();
if (gamepad1.dpad_down && !prevDpadDown) tuning.selectNext();
if (gamepad1.dpad_left && !prevDpadLeft) applyChange(tuning.adjust(-1, coarse));
if (gamepad1.dpad_right && !prevDpadRight) applyChange(tuning.adjust(1, coarse));
```

`applyChange` must call `alignment.setConfig(...)` for `ALIGNMENT`, or `vision.setBallTargetingConfig(...)` for `TARGETING`; both paths command zero for that loop.

- [ ] **Step 2: Add the shared alignment loop and safety transitions**

Reuse the MANUAL/ALIGN behavior and color-selection semantics. Use only:

```java
BallAlignmentController.Result result =
        alignment.update(vision.getTarget(), System.currentTimeMillis(), loopDt);
drivetrain.driveRaw(0.0, 0.0, result.getAppliedTurn());
```

Configuration changes, B/manual, color changes, target invalidation, and shutdown must call the appropriate reset and produce zero before another motor command.

- [ ] **Step 3: Add complete Driver Station display**

Always show:

- selected parameter, fine/coarse increment, selected current/default values;
- all 12 current runtime values;
- requested color;
- fresh/held/invalid target and age;
- lock/pending sizes and confirmation count;
- raw/filtered/diagnostic mean bearings;
- controller action and correction state;
- P/I/D, raw PID, requested/applied power;
- friction, slew, expiry, and reversal flags;
- existing Limelight pipeline/staleness/latency/detection telemetry.

- [ ] **Step 4: Document the OpModes and controls**

Add both `Ball-Grouping-Aligning test` and `Ball-Group Alignment Tuner` to `docs/OPMODES.md`. State that tuner values are runtime-only and must be copied to defaults after validation.

- [ ] **Step 5: Compile TeamCode**

```powershell
.\gradlew.bat :TeamCode:compileDebugJavaWithJavac
```

Expected: PASS and both Test OpModes register without annotation/name conflicts.

- [ ] **Step 6: Commit**

```powershell
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/BallGroupAlignmentTuner.java docs/OPMODES.md
git commit -m "feat: add Driver Station ball alignment tuner"
```

---

### Task 9: Full Verification and On-Robot Acceptance

**Files:**
- Verify all files named above.
- No source changes unless a verification failure identifies a defect in this scope.

**Interfaces:**
- Confirms the complete perception → persistence → control → drivetrain path.
- Confirms AprilTag code and constants remain unchanged.

- [ ] **Step 1: Run all TeamCode unit tests**

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest
```

Expected: PASS.

- [ ] **Step 2: Compile TeamCode**

```powershell
.\gradlew.bat :TeamCode:compileDebugJavaWithJavac
```

Expected: PASS.

- [ ] **Step 3: Check formatting and scope**

```powershell
git diff --check
git diff --name-only HEAD~8
rg -n "VISION_SEEK_" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Constants.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/LimelightSeekTag.java
```

Expected: no whitespace errors; changed source is limited to the planned ball alignment, ball vision, tests, constants, and OpMode documentation; existing AprilTag values and construction remain unchanged.

- [ ] **Step 4: Perform stationary-target acceptance**

On Driver Station:

1. Open `Ball-Group Alignment Tuner`.
2. Measure and set the lowest static-friction power that rotates all four wheels.
3. Place one group about 15 degrees off center and enter ALIGN.
4. Verify prompt rotation, maximum applied command at or below 0.65, and sustained zero inside 2 degrees.
5. Occlude the group and verify command expiry begins after 100 ms, followed by immediate zero when invalid.

- [ ] **Step 5: Perform multi-group acceptance**

1. Present a nearer two-ball group and a farther three-ball group; verify the three-ball group wins.
2. Present equal-count groups; verify the apparently closer group wins.
3. Make another group win for one or two fresh frames; verify no switch.
4. Sustain its win for three fresh frames; verify exactly one deliberate switch.
5. Momentarily change the locked group's count or center; verify the lock remains and the filtered bearing changes smoothly.

- [ ] **Step 6: Record tuned defaults**

Copy only field-validated tuner values into the corresponding `BALL_ALIGN_*` or `BALL_TARGET_*` defaults, rerun Steps 1–3, and commit that calibration separately:

```powershell
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Constants.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/BallVisionConfig.java
git commit -m "tune: calibrate ball alignment defaults"
```

Skip this commit when the approved defaults require no changes.
