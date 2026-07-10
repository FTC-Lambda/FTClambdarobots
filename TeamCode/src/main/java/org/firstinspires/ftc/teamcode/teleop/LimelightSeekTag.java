package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.VisionDeadband;

import java.util.List;

/**
 * Manual + auto test OpMode for the Limelight 3A that seeks a SPECIFIC, hand-picked tag.
 *
 * CONTROLS:
 *   - Left/right sticks: manual driving (only in MANUAL).
 *   - D-pad left / right: change the target AprilTag ID (20, 21, 22, 23, 24) — MANUAL only.
 *   - D-pad up / down: raise / lower the follow standoff distance (any time).
 *   - Right / left bumper: raise / lower the spin (turn) power (any time).
 *   - X / Y: raise / lower the turn dead-zone (any time).
 *   - Left / right trigger: lower / raise distance deadband (any time).
 *   - Start: reset deadbands/dead-zone to Constants defaults.
 *   - A: start FOLLOW — continuously track the SELECTED tag: turn to keep it centered while
 *        holding the standoff distance, so it keeps following as the tag moves (drives up if
 *        too far, backs off if too close). If the tag goes out of view it spins to reacquire.
 *   - B: stop / return to MANUAL at any time.
 *
 * TURN CONTROL: a continuous PD loop on bearing error (no on/off hysteresis, no hard power
 * floor) — TURN_KP drives toward the target, TURN_KD damps the approach using the tag's
 * smoothed angular velocity so the turn tapers to zero instead of overshooting past center.
 * A small feedforward (TURN_KS) is added on top to overcome wheel static friction without
 * breaking the continuity of the command. DRIVE (forward/back) still uses a velocity
 * feedforward the same way — it leads a moving tag instead of always lagging behind it.
 *
 * Pipeline 0 must be a fiducial pipeline with the 3D solve enabled (run "Limelight
 * Pipeline Setup" if distance reads 0 or the pipeline shows as color).
 */
@TeleOp(name = "Limelight Seek Tag", group = "Test")
public class LimelightSeekTag extends LinearOpMode {

	static final int PIPELINE_APRILTAG = 0; // Pipeline 0 must be a Fiducial/AprilTag pipeline

	// Selectable target tag IDs.
	static final int[] TAG_CHOICES = {20, 21, 22, 23, 24};

	// --- Auto tuning ---
	// Turn control is a PD loop on bearing error: turnCmd = bearing*TURN_KP + bearingRate*TURN_KD.
	// bearingRate is the already-smoothed d(bearing)/dt, so TURN_KD is a genuine damping term
	// (it opposes the P term as the error closes), not just a "lead" feedforward.
	static final double TURN_KP           = 0.03;  // power per degree of bearing error
	// Damping term, conservative starting value: at TURN_KP=0.03 a 5 deg error alone
	// contributes 0.15 power, so TURN_KD is kept well below the point where a moderate
	// closing rate (~10 deg/s) could swamp or reverse the P term. Field-tune upward
	// carefully if the approach still overshoots.
	static final double TURN_KD           = 0.02;  // power per (deg/s) of bearing closing rate — damping
	// Static-friction feedforward: ADDED to the PD output (never replaces it), so the total
	// command stays continuous instead of jumping to a fixed floor. Ramped in linearly over
	// TURN_KS_RAMP_DEG past the dead-zone edge (see below) rather than snapping straight to
	// full strength, which would itself be a smaller version of the same discontinuity this
	// whole rework is trying to eliminate.
	static final double TURN_KS           = 0.10;
	static final double TURN_KS_RAMP_DEG  = 2.0;
	// True dead-zone: below this, treat bearing noise as centered and command zero. This
	// replaces the old wide on/off hysteresis band — a tight epsilon plus a continuous PD
	// taper avoids the earlier discontinuity (which caused the robot to overshoot past
	// center and oscillate) while still stopping noise-driven buzzing at dead center.
	static final double TURN_EPSILON_DEG  = 1.0;
	static final double TURN_EPSILON_STEP = 0.25; // X/Y adjustment step
	static final double TURN_EPSILON_MIN  = 0.25;
	static final double TURN_EPSILON_MAX  = 5.0;
	static final double AIM_FALLOFF_DEG   = 30.0; // deg — forward drive fades to 0 by this bearing
	static final double MAX_DRIVE         = 0.5;  // cap on forward/back power
	static final double DRIVE_GAIN        = 0.03; // drive power per inch of distance error

	// --- Adjustable spin (turn) power ---
	static final double SPIN_POWER_DEFAULT = 0.5; // caps proportional turn AND sets search speed
	static final double SPIN_STEP          = 0.05;
	static final double SPIN_MIN           = 0.1;
	static final double SPIN_MAX           = 1.0;

	// How long the tag must be missing before switching to a full-speed search spin. A
	// single missed frame is routine (motion blur, momentary occlusion) and should not
	// interrupt tracking. Time-based rather than frame-counted since the OpMode loop rate
	// and the Limelight's poll rate aren't guaranteed to be in lockstep.
	static final double LOST_DEBOUNCE_MS = 150.0;

	// --- Predictive tracking (velocity feedforward for the drive/standoff axis) ---
	// Turn's velocity term is TURN_KD above (it does double duty as PD damping); this one is
	// drive-only.
	static final double DRIVE_VEL_GAIN = 0.020; // drive power per (in/s) of tag range speed
	static final double RATE_SMOOTHING = 0.4;   // 0..1 low-pass on the velocity estimates (higher = snappier)

	// --- Standoff distance (how far to hold from the tag) ---
	static final double DEFAULT_STANDOFF_IN = 40.0; // starting standoff
	static final double STANDOFF_STEP_IN    = 2.0;  // D-pad up/down increment
	static final double STANDOFF_MIN_IN     = 6.0;
	static final double STANDOFF_MAX_IN     = 120.0;

	private enum Mode { MANUAL, FOLLOW }

	@Override
	public void runOpMode() {
		RobotHardware robot = new RobotHardware();
		robot.init(hardwareMap);
		Drivetrain drivetrain = new Drivetrain(robot);

		robot.limelight.pipelineSwitch(PIPELINE_APRILTAG);
		robot.limelight.setPollRateHz(100);
		robot.limelight.start();

		telemetry.addLine("Limelight Seek Tag ready — press START");
		telemetry.update();
		waitForStart();

		Mode mode = Mode.MANUAL;
		int choiceIndex = 0;                 // index into TAG_CHOICES
		double standoffIn = DEFAULT_STANDOFF_IN;
		double spinPower = SPIN_POWER_DEFAULT;
		double turnEpsilonDeg = TURN_EPSILON_DEG;
		VisionDeadband deadband = new VisionDeadband();
		boolean prevDpadLeft = false, prevDpadRight = false;
		boolean prevDpadUp = false, prevDpadDown = false;
		boolean prevRB = false, prevLB = false;
		boolean prevX = false, prevY = false;
		boolean prevLT = false, prevRT = false;
		boolean prevStart = false;

		// Predictive-tracking state (tag motion estimated between frames).
		ElapsedTime rateTimer = new ElapsedTime();
		boolean hasPrev = false;
		double prevBearing = 0.0, prevRange = 0.0;
		double bearingRate = 0.0, rangeRate = 0.0; // smoothed deg/s and in/s

		// Debounce for treating the tag as lost (see LOST_DEBOUNCE_MS). Reset any time the
		// tag is seen or FOLLOW is (re)entered; left running while the tag is missing.
		ElapsedTime timeSinceSeen = new ElapsedTime();

		while (opModeIsActive()) {
			// --- Buttons ---
			if (gamepad1.b) {
				mode = Mode.MANUAL;
				deadband.clearState();
			} else if (gamepad1.a && mode == Mode.MANUAL) {
				mode = Mode.FOLLOW;
				deadband.clearState();
				timeSinceSeen.reset();
			}

			// Change target tag with D-pad (rising edge), MANUAL only.
			if (mode == Mode.MANUAL) {
				if (gamepad1.dpad_right && !prevDpadRight) {
					choiceIndex = (choiceIndex + 1) % TAG_CHOICES.length;
				}
				if (gamepad1.dpad_left && !prevDpadLeft) {
					choiceIndex = (choiceIndex - 1 + TAG_CHOICES.length) % TAG_CHOICES.length;
				}
			}
			prevDpadLeft = gamepad1.dpad_left;
			prevDpadRight = gamepad1.dpad_right;

			// Change standoff distance with D-pad up/down (rising edge), any time.
			if (gamepad1.dpad_up && !prevDpadUp) {
				standoffIn = Math.min(STANDOFF_MAX_IN, standoffIn + STANDOFF_STEP_IN);
			}
			if (gamepad1.dpad_down && !prevDpadDown) {
				standoffIn = Math.max(STANDOFF_MIN_IN, standoffIn - STANDOFF_STEP_IN);
			}
			prevDpadUp = gamepad1.dpad_up;
			prevDpadDown = gamepad1.dpad_down;

			// Adjust spin power with bumpers (rising edge), any time.
			if (gamepad1.right_bumper && !prevRB) {
				spinPower = Math.min(SPIN_MAX, spinPower + SPIN_STEP);
			}
			if (gamepad1.left_bumper && !prevLB) {
				spinPower = Math.max(SPIN_MIN, spinPower - SPIN_STEP);
			}
			prevRB = gamepad1.right_bumper;
			prevLB = gamepad1.left_bumper;

			// Turn dead-zone (epsilon): X up, Y down (rising edge).
			if (gamepad1.x && !prevX) {
				turnEpsilonDeg = Math.min(TURN_EPSILON_MAX, turnEpsilonDeg + TURN_EPSILON_STEP);
			}
			if (gamepad1.y && !prevY) {
				turnEpsilonDeg = Math.max(TURN_EPSILON_MIN, turnEpsilonDeg - TURN_EPSILON_STEP);
			}
			prevX = gamepad1.x;
			prevY = gamepad1.y;

			// Distance deadband: right trigger up, left trigger down (rising edge past 50%).
			boolean lt = gamepad1.left_trigger > 0.5;
			boolean rt = gamepad1.right_trigger > 0.5;
			if (rt && !prevRT) {
				deadband.nudgeDistanceDeadband(Constants.VISION_DISTANCE_DEADBAND_STEP);
			}
			if (lt && !prevLT) {
				deadband.nudgeDistanceDeadband(-Constants.VISION_DISTANCE_DEADBAND_STEP);
			}
			prevLT = lt;
			prevRT = rt;

			if (gamepad1.start && !prevStart) {
				deadband.resetToDefaults();
				turnEpsilonDeg = TURN_EPSILON_DEG;
			}
			prevStart = gamepad1.start;

			int targetTagId = TAG_CHOICES[choiceIndex];

			// --- Read tags once ---
			LLResult result = robot.limelight.getLatestResult();
			List<LLResultTypes.FiducialResult> tags =
					(result != null && result.isValid()) ? result.getFiducialResults() : null;
			LLResultTypes.FiducialResult target = findTag(tags, targetTagId);

			double drivePower = 0.0, turnPower = 0.0;
			String action;

			switch (mode) {
				case FOLLOW:
					if (target != null) {
						timeSinceSeen.reset();
						double bearing = target.getTargetXDegrees();
						double range = rangeInches(target);

						// --- Estimate the tag's motion between frames (smoothed) ---
						double dt = rateTimer.seconds();
						if (hasPrev && dt > 1e-3) {
							double rawBearingRate = (bearing - prevBearing) / dt; // deg/s
							double rawRangeRate = (range - prevRange) / dt;       // in/s
							bearingRate = RATE_SMOOTHING * rawBearingRate + (1 - RATE_SMOOTHING) * bearingRate;
							rangeRate = RATE_SMOOTHING * rawRangeRate + (1 - RATE_SMOOTHING) * rangeRate;
						}
						prevBearing = bearing;
						prevRange = range;
						hasPrev = true;
						rateTimer.reset();

						// PD control on bearing error, continuous all the way to the setpoint — no
						// on/off hysteresis gate and no hard floor override (both of those created
						// power discontinuities that reliably swung the robot past center and back,
						// which is what caused the sustained left/right jitter). TURN_KD is the
						// damping term: as the error closes, bearingRate (already low-pass filtered)
						// naturally opposes the P term and tapers the command toward zero.
						if (Math.abs(bearing) > turnEpsilonDeg) {
							double turnCmd = bearing * TURN_KP + bearingRate * TURN_KD;
							turnCmd = clamp(turnCmd, -spinPower, spinPower);
							// Static-friction feedforward: ADDED on top of the PD output (not a
							// floor that replaces it), so the command stays continuous. Ramped in
							// linearly over TURN_KS_RAMP_DEG past the dead-zone edge instead of
							// snapping straight to full strength right at turnEpsilonDeg — a hard
							// 0-to-TURN_KS jump at that boundary would itself be a smaller copy of
							// the discontinuity that caused the original jitter.
							double rampScale = clamp(
									(Math.abs(bearing) - turnEpsilonDeg) / TURN_KS_RAMP_DEG, 0.0, 1.0);
							turnCmd += Math.copySign(Math.min(TURN_KS, spinPower) * rampScale, bearing);
							turnPower = clamp(turnCmd, -spinPower, spinPower);
						}

						// Hold standoff; only drive when outside distance deadband.
						double distError = range - standoffIn; // + = too far
						double aimFactor = clamp(1.0 - Math.abs(bearing) / AIM_FALLOFF_DEG, 0.0, 1.0);
						double driveCmd;
						if (!deadband.shouldCorrectDrive(distError)) {
							driveCmd = rangeRate * DRIVE_VEL_GAIN; // at setpoint: just track motion
							action = String.format("FOLLOW holding %.0f in (tag %d)", standoffIn, targetTagId);
						} else {
							driveCmd = distError * DRIVE_GAIN + rangeRate * DRIVE_VEL_GAIN;
							action = String.format("FOLLOW %s (tag %d)",
									distError > 0 ? "approaching" : "backing off", targetTagId);
						}
						drivePower = clamp(driveCmd, -MAX_DRIVE, MAX_DRIVE) * aimFactor;
					} else if (timeSinceSeen.milliseconds() < LOST_DEBOUNCE_MS) {
						// Momentarily missing (motion blur, momentary occlusion) — sit still rather
						// than immediately slamming into a full-speed search spin, which tends to
						// overshoot right past the tag. turnPower/drivePower stay at their 0.0
						// defaults for this iteration. Velocity feedforward (bearingRate/rangeRate)
						// is deliberately left alone here since the tag may reappear next frame and
						// the estimate is still fresh.
						action = String.format("FOLLOW coasting (%.0f ms since seen)", timeSinceSeen.milliseconds());
					} else {
						// Genuinely lost the tag — spin to reacquire it (keeps following once it
						// reappears). Clear both the frame-to-frame link AND the smoothed velocity
						// estimate so reacquisition starts from a clean state instead of applying a
						// stale feedforward left over from before the tag went missing.
						turnPower = spinPower;
						drivePower = 0.0;
						hasPrev = false;
						bearingRate = 0.0;
						rangeRate = 0.0;
						action = "FOLLOW searching for tag " + targetTagId;
					}
					break;

				case MANUAL:
				default:
					hasPrev = false; // start FOLLOW with a fresh velocity estimate
					bearingRate = 0.0;
					rangeRate = 0.0;
					deadband.clearState();
					action = "MANUAL";
					break;
			}

			// --- Apply drive ---
			if (mode == Mode.MANUAL) {
				drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
			} else {
				drivetrain.driveRaw(drivePower, 0, turnPower);
			}

			// --- Status 1: connection + pipeline sanity ---
			telemetry.addData("1) Limelight", robot.limelight.isConnected() ? "CONNECTED" : "NOT CONNECTED");
			LLStatus status = robot.limelight.getStatus();
			if (status != null) {
				String type = status.getPipelineType();
				telemetry.addData("   Pipeline", "#%d (%s)", status.getPipelineIndex(), type);
				if (type != null && !type.toLowerCase().contains("fiducial")) {
					telemetry.addLine("   ! Pipeline 0 is NOT fiducial — run Limelight Pipeline Setup");
				}
			}

			// --- Target selection + live tag details ---
			telemetry.addLine();
			telemetry.addData("Target tag (D-pad L/R)", targetTagId);
			telemetry.addData("Standoff (D-pad U/D)", "%.0f in", standoffIn);
			telemetry.addData("Spin power (bumpers)", "%.2f", spinPower);
			telemetry.addData("Turn dead-zone (X/Y)", "%.2f deg", turnEpsilonDeg);
			telemetry.addData("Distance deadband (LT/RT)", "%.1f in", deadband.getDistanceDeadbandIn());
			telemetry.addLine("Start = reset deadbands to defaults");
			if (target != null) {
				double dist = rangeInches(target);
				telemetry.addData("2) Target tag", "IN VIEW");
				telemetry.addData("3) Distance (in)", "%.1f", dist);
				telemetry.addData("   Dist error (in)", "%.1f", dist - standoffIn);
				telemetry.addData("   Bearing (deg)", "%.1f", target.getTargetXDegrees());
			} else {
				telemetry.addData("2) Target tag", "NOT IN VIEW");
				telemetry.addData("3) Distance (in)", "--");
			}

			// --- Auto debug indicators ---
			telemetry.addLine();
			telemetry.addData("MODE", mode + (mode == Mode.MANUAL ? " (A=start)" : " (B=stop)"));
			telemetry.addData("Action", action);
			telemetry.addData("Target visible?", target != null ? "YES" : "no");
			telemetry.addData("Centered?", target != null
					? (Math.abs(target.getTargetXDegrees()) <= turnEpsilonDeg ? "YES" : "no") : "--");
			telemetry.addData("Tag speed (deg/s, in/s)", "%.0f / %.0f", bearingRate, rangeRate);
			telemetry.addData("Cmd drive / turn", "%.2f / %.2f", drivePower, turnPower);
			telemetry.update();
		}

		robot.limelight.stop();
	}

	/** Straight-ahead distance (in) from the camera to a tag; 0 if the pose is unavailable. */
	private double rangeInches(LLResultTypes.FiducialResult f) {
		return Math.abs(f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH).z);
	}

	/** Finds the fiducial with the given ID in the current frame, or null. */
	private LLResultTypes.FiducialResult findTag(List<LLResultTypes.FiducialResult> tags, int tagId) {
		if (tags == null) return null;
		for (LLResultTypes.FiducialResult f : tags) {
			if (f.getFiducialId() == tagId) return f;
		}
		return null;
	}

	private static double clamp(double v, double lo, double hi) {
		return Math.max(lo, Math.min(hi, v));
	}
}
