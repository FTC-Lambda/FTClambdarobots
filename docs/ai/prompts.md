AI Prompts — Reusable Templates

Use these templates when asking for research, code changes, reviews, or debugging help. Replace placeholders with concrete file paths, class names, or values.

- **Research / background**
  - "Summarize the best practices for AprilTag detection using Limelight 3A on FTC Control Hub. Include required pipelines, typical exposure settings, and performance trade-offs."

- **Implementation (feature request)**
  - "Implement `driveRaw(double y, double x, double rx)` in `TeamCode/src/.../Drivetrain.java`. Requirements: no input squaring, normalize motor outputs, apply per-motor scale factors from `Constants.java`, and include safety clamps. Provide only the new method."

- **Code review**
  - "Review the following OpMode for safety and performance: [file path]. Check for long-running blocking calls, missing `opModeIsActive()` checks, telemetry frequency, and constant placement. Return a short list of recommended fixes."

- **Debug prompt**
  - "Robot shows encoder drift during autonomous. Here are telemetry logs: [paste]. Explain likely causes and prioritized fixes. Suggest specific telemetry to add for next run."

- **Testing prompt**
  - "Create a minimal on-block test procedure to validate strafing accuracy over 2 meters on carpet. Include measurement steps, expected tolerances, and telemetry to collect."

- **Agent task decomposition**
  - "Break implementing AprilTag SEEK autonomous into discrete tasks (max 10), each with a single-file change or test. Identify which tasks require hardware testing." 

Guidance:
- Keep prompts explicit about file paths and constraints.
- Provide expected output format (e.g., code-only, bullet list, JSON) to help downstream parsing.
