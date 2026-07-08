# AI Prompts — Lambda Robotics

Reusable prompts for Codex / ChatGPT workflows.

Use these when asking for repo analysis, debugging, implementation, review, or robot testing.

Rules:
- Replace placeholders before use.
- Give file paths when known.
- Ask for diagnosis before code changes.
- Prefer small changes over broad refactors.
- Do not let agents invent hardware names.

---

## 1. Repo Analyst

Use this before coding.

```text
Read AGENTS.md and docs/ai.

Inspect the repo. Do not edit files.

Find:
1. Relevant OpModes
2. Relevant subsystems
3. Hardware mapping files
4. Constants/config files
5. Existing utilities
6. Existing libraries/patterns
7. Existing reusable code that should not be duplicated

Return only:
- relevant files
- current architecture summary
- reusable code found
- risks before editing
```

---

## 2. Research Agent

Use when you want the AI to check known FTC/library patterns before implementation.

```text
Research the best existing FTC approaches for this task:

Task:
[describe feature or bug]

Prioritize:
1. Existing repo code
2. FTC SDK patterns
3. Road Runner or Pedro Pathing if pathing-related
4. FTCLib if control/command-related
5. Limelight / FTC VisionPortal / EasyOpenCV only if vision-related

Do not write code yet.

Return:
1. Relevant known approaches
2. What applies to this repo
3. What should not be copied blindly
4. Recommended minimal implementation strategy
5. Risks
```

---

## 3. Debug Investigator

Use when the robot behaves wrong and the cause is unknown.

```text
Problem:
[describe observed robot behavior]

Do not assume my diagnosis is correct.

Read:
- AGENTS.md
- docs/ai/debugging.md
- relevant source files

Inspect relevant code.
Do not edit files yet.

Return:
1. Likely causes ranked by probability
2. Evidence level for each cause:
   - Code-confirmed
   - Build-confirmed
   - Telemetry-supported
   - Human-observed
   - Unconfirmed hypothesis
3. Telemetry needed
4. Smallest diagnostic change
5. What requires physical robot testing
```

---

## 4. Implementation Agent

Use only after a plan is approved.

```text
Implement the approved plan only.

Plan:
[paste approved plan]

Rules:
- Modify the fewest files possible.
- Reuse existing code.
- Do not invent hardware names.
- Do not refactor unrelated files.
- Preserve TeleOp responsiveness.
- No blocking sleeps in active robot loops.
- Clamp control outputs.
- Handle stop requests.
- Handle null/stale vision data.
- Add telemetry only where useful.

Run:
./gradlew assembleDebug

Return:
1. Summary
2. Files changed
3. Verification result
4. Robot testing checklist
5. Risks/tuning values
```

---

## 5. Code Review Agent

Use after Codex changes code.

```text
Review the current diff as an FTC competition code reviewer.

Focus on:
- compile issues
- FTC SDK misuse
- hardware map mistakes
- blocking loops
- missing stop handling
- missing motor stop path
- null vision handling
- stale vision handling
- unclamped outputs
- PID instability
- unsafe autonomous behavior
- unrelated refactors

Ignore style-only comments unless they affect reliability.

Return:
1. Must-fix issues
2. Should-fix issues
3. Safe to robot-test? yes/no
4. Reason
```

---

## 6. Robot Test Planner

Use before physical robot testing.

```text
Create a robot test checklist for this change.

Change:
[briefly describe change]

Include:
1. On-blocks test
2. Low-power carpet test
3. Full-speed test if safe
4. Telemetry values to watch
5. Expected behavior
6. Emergency stop behavior
7. Failure signs
8. Rollback condition

Do not suggest more code unless required for safety.
```

---

## 7. Unknown Bug Prompt

Use when the team does not know what is wrong.

```text
Observed symptom:
[describe exactly what the robot does]

Context:
[TeleOp / autonomous / vision / drivetrain / mechanism]

Do not assume our explanation is correct.

First diagnose only.

Read:
- AGENTS.md
- docs/ai/debugging.md
- relevant source files

Return:
1. What code areas are involved
2. Top 5 likely causes
3. Evidence supporting each cause
4. Evidence missing
5. Telemetry to collect next
6. Safest first diagnostic test
7. Whether a code change is justified yet
```

---

## 8. AprilTag / Limelight Debug Prompt

Use for AprilTag seeking, centering, or approach problems.

```text
Problem:
[describe AprilTag behavior]

Do not edit code yet.

Inspect:
- Limelight setup
- AprilTag OpModes
- drivetrain control
- constants
- telemetry

Check for:
- wrong pipeline
- wrong accepted tag ID
- null or stale detections
- sign convention errors
- range/yaw/tx misunderstanding
- output not clamped
- PID oscillation
- combining turn/strafe/forward too aggressively
- missing timeout
- missing motor stop path

Return:
1. Likely causes ranked
2. Code-confirmed issues
3. Telemetry needed
4. Smallest diagnostic patch
5. Safe robot test procedure
```

---

## 9. Small Feature Request

Use for isolated changes.

```text
Implement this small feature:

Feature:
[describe feature]

Files likely involved:
[list files if known]

Constraints:
- Do not modify unrelated files.
- Reuse existing constants/utilities.
- Do not create a new architecture.
- Keep the implementation minimal.
- Run ./gradlew assembleDebug.

Return:
1. Summary
2. Files changed
3. Verification result
4. Risks
```

---

## 10. Prompt Builder

Use when you do not know how to ask Codex properly.

```text
Help me write the best Codex prompt for this FTC task.

Task:
[describe what I want]

Known context:
[hardware, subsystem, bug symptoms, files, logs, or constraints]

I may be wrong about the cause, so make the prompt force Codex to inspect and diagnose first.

Return:
1. Final prompt to paste into Codex
2. Files Codex should inspect
3. What Codex should not change
4. Expected output format
```