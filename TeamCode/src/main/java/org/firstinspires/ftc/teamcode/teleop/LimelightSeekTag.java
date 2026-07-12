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
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
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
 *   - X / Y: raise / lower bearing deadband (any time).
 *   - Left / right trigger: lower / raise distance deadband (any time).
 *   - Start: reset deadbands to Constants defaults.
 *   - A: start FOLLOW — continuously track the SELECTED tag: turn to keep it centered while
 *        holding the standoff distance, so it keeps following as the tag moves (drives up if
 *        too far, backs off if too close). If the tag goes out of view it spins to reacquire.
 *   - B: stop / return to MANUAL at any time.
 *
 * TRACKING: bearing and range are each closed-loop PID (see util.PIDController) rather than
 * plain proportional control — the D term is computed from the raw measurement (not the
 * error), so it acts as a filtered velocity feedforward that leads a moving tag, and the I
 * term corrects small persistent offsets (e.g. camera mounting bias) that pure P leaves
 * uncorrected. Final drive/turn power is slew-rate limited (util.SlewRateLimiter) so
 * corrections ramp in smoothly instead of snapping.
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
	static final double TURN_MIN_POWER    = 0.2;  // feedforward: smallest turn that spins ALL wheels
	static final double AIM_FALLOFF_DEG   = 30.0; // deg — forward drive fades to 0 by this bearing
	static final double MAX_DRIVE         = 0.5;  // cap on forward/back power

	// --- Adjustable spin (turn) power ---
	static final double SPIN_POWER_DEFAULT = 0.5; // caps proportional turn AND sets search speed
	static final double SPIN_STEP          = 0.05;
	static final double SPIN_MIN           = 0.1;
	static final double SPIN_MAX           = 1.0;

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
		VisionDeadband deadband = new VisionDeadband();
		boolean prevDpadLeft = false, prevDpadRight = false;
		boolean prevDpadUp = false, prevDpadDown = false;
		boolean prevRB = false, prevLB = false;
		boolean prevX = false, prevY = false;
		boolean prevLT = false, prevRT = false;
		boolean prevStart = false;

		// Closed-loop tracking: PID per axis (P+I+D — D also serves as velocity feedforward
		// on a moving tag), plus a slew limiter on the final commanded power for smoothness.
		PIDController turnPid = new PIDController(
				Constants.VISION_SEEK_TURN_KP, Constants.VISION_SEEK_TURN_KI, Constants.VISION_SEEK_TURN_KD,
				Constants.VISION_SEEK_TURN_INTEGRAL_LIMIT, Constants.VISION_SEEK_TURN_INTEGRAL_ZONE_DEG,
				Constants.VISION_TURN_DERIVATIVE_FILTER);
		PIDController drivePid = new PIDController(
				Constants.VISION_SEEK_DRIVE_KP, Constants.VISION_SEEK_DRIVE_KI, Constants.VISION_SEEK_DRIVE_KD,
				Constants.VISION_SEEK_DRIVE_INTEGRAL_LIMIT, Constants.VISION_SEEK_DRIVE_INTEGRAL_ZONE_IN,
				Constants.VISION_DRIVE_DERIVATIVE_FILTER);
		SlewRateLimiter turnSlew = new SlewRateLimiter(Constants.VISION_TURN_SLEW_RATE);
		SlewRateLimiter driveSlew = new SlewRateLimiter(Constants.VISION_DRIVE_SLEW_RATE);
		ElapsedTime loopTimer = new ElapsedTime();

		while (opModeIsActive()) {
			double dt = loopTimer.seconds();
			loopTimer.reset();

			// --- Buttons ---
			if (gamepad1.b) {
				mode = Mode.MANUAL;
				deadband.clearState();
				turnPid.reset();
				drivePid.reset();
			} else if (gamepad1.a && mode == Mode.MANUAL) {
				mode = Mode.FOLLOW;
				deadband.clearState();
				turnPid.reset();
				drivePid.reset();
				turnSlew.reset();
				driveSlew.reset();
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

			// Bearing deadband: X up, Y down (rising edge).
			if (gamepad1.x && !prevX) {
				deadband.nudgeBearingDeadband(Constants.VISION_BEARING_DEADBAND_STEP);
			}
			if (gamepad1.y && !prevY) {
				deadband.nudgeBearingDeadband(-Constants.VISION_BEARING_DEADBAND_STEP);
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
						double bearing = target.getTargetXDegrees();
						double range = rangeInches(target);

						// PID runs every frame regardless of the deadband so its integral/
						// derivative state stays current; whether the output gets applied
						// depends on the hysteresis deadband below.
						double turnOut = turnPid.calculate(bearing, 0.0, dt);

						// Hysteresis deadband: hold turn still inside the band to avoid jitter.
						if (deadband.shouldCorrectTurn(bearing)) {
							turnPower = clamp(turnOut, -spinPower, spinPower);
							if (Math.abs(turnPower) < TURN_MIN_POWER) {
								turnPower = Math.copySign(TURN_MIN_POWER, bearing);
							}
						}

						// Hold standoff; only drive when outside distance deadband.
						double distError = range - standoffIn; // + = too far
						double driveOut = drivePid.calculate(range, standoffIn, dt);
						double aimFactor = clamp(1.0 - Math.abs(bearing) / AIM_FALLOFF_DEG, 0.0, 1.0);
						double driveCmd;
						if (!deadband.shouldCorrectDrive(distError)) {
							driveCmd = drivePid.getLastD(); // at setpoint: D term alone tracks tag motion
							action = String.format("FOLLOW holding %.0f in (tag %d)", standoffIn, targetTagId);
						} else {
							driveCmd = driveOut;
							action = String.format("FOLLOW %s (tag %d)",
									distError > 0 ? "approaching" : "backing off", targetTagId);
						}
						drivePower = clamp(driveCmd, -MAX_DRIVE, MAX_DRIVE) * aimFactor;
					} else {
						// Lost the tag — spin to reacquire it (keeps following once it reappears).
						turnPower = spinPower;
						drivePower = 0.0;
						turnPid.reset(); // don't carry stale state across the gap
						drivePid.reset();
						action = "FOLLOW searching for tag " + targetTagId;
					}
					break;

				case MANUAL:
				default:
					action = "MANUAL";
					break;
			}

			// --- Apply drive (slew-limited in FOLLOW so corrections ramp instead of snapping) ---
			double smoothedDrive = drivePower;
			double smoothedTurn = turnPower;
			if (mode == Mode.MANUAL) {
				drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
			} else {
				smoothedDrive = driveSlew.calculate(drivePower, dt);
				smoothedTurn = turnSlew.calculate(turnPower, dt);
				drivetrain.driveRaw(smoothedDrive, 0, smoothedTurn);
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
			telemetry.addData("Bearing deadband (X/Y)", "%.1f deg", deadband.getBearingDeadbandDeg());
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
					? (deadband.isBearingCentered(target.getTargetXDegrees()) ? "YES" : "no") : "--");
			telemetry.addData("Turn PID (P/I/D)", "%.2f / %.2f / %.2f",
					turnPid.getLastP(), turnPid.getLastI(), turnPid.getLastD());
			telemetry.addData("Drive PID (P/I/D)", "%.2f / %.2f / %.2f",
					drivePid.getLastP(), drivePid.getLastI(), drivePid.getLastD());
			telemetry.addData("Cmd drive / turn (raw)", "%.2f / %.2f", drivePower, turnPower);
			telemetry.addData("Cmd drive / turn (smoothed)", "%.2f / %.2f", smoothedDrive, smoothedTurn);
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
