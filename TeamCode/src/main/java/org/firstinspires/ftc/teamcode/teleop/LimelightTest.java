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

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Manual + auto test OpMode for the Limelight 3A.
 *
 * CONTROLS:
 *   - Left/right sticks: manual driving (only when in MANUAL).
 *   - A: start the auto sequence:
 *        1) SCAN  — spin ~360 degrees, record every AprilTag seen and its closest distance.
 *        2) TURN  — spin toward the CLOSEST tag found and center on it.
 *        3) DRIVE — drive straight forward (no safety — keeps going even if the tag is lost).
 *   - B: stop / return to MANUAL at any time.
 *
 * Pipeline 0 must be a fiducial pipeline with the 3D solve enabled (run "Limelight
 * Pipeline Setup" if distance reads 0 or the pipeline shows as color).
 */
@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends LinearOpMode {

	static final int PIPELINE_APRILTAG = 0; // Pipeline 0 must be a Fiducial/AprilTag pipeline

	// --- Auto tuning ---
	static final double SCAN_POWER            = 0.3;   // turn power during the 360 scan
	static final long   SCAN_DURATION_MS      = 2500;  // TUNE: time for ~one full rotation
	static final double TURN_GAIN             = 0.02;  // turn power per degree of bearing error
	static final double MAX_TURN              = 0.3;   // cap on turn power
	static final double BEARING_THRESHOLD     = 5.0;   // deg — "facing the tag" -> start driving
	static final double FORWARD_POWER         = 0.3;   // forward power in DRIVE

	private enum Mode { MANUAL, SCAN, TURN, DRIVE }

	@Override
	public void runOpMode() {
		RobotHardware robot = new RobotHardware();
		robot.init(hardwareMap);
		Drivetrain drivetrain = new Drivetrain(robot);

		robot.limelight.pipelineSwitch(PIPELINE_APRILTAG);
		robot.limelight.setPollRateHz(100);
		robot.limelight.start();

		telemetry.addLine("Limelight Test ready — press START");
		telemetry.update();
		waitForStart();

		Mode mode = Mode.MANUAL;
		ElapsedTime scanTimer = new ElapsedTime();
		Map<Integer, Double> closestByTag = new HashMap<>(); // tag ID -> closest distance (in)
		int targetTagId = -1;
		double targetTagDistance = 0.0;
		String note = "";

		while (opModeIsActive()) {
			// --- Buttons: A starts the sequence from MANUAL, B always aborts ---
			if (gamepad1.b) {
				mode = Mode.MANUAL;
			} else if (gamepad1.a && mode == Mode.MANUAL) {
				mode = Mode.SCAN;
				scanTimer.reset();
				closestByTag.clear();
				targetTagId = -1;
				note = "";
			}

			// --- Read tags once ---
			LLResult result = robot.limelight.getLatestResult();
			List<LLResultTypes.FiducialResult> tags =
					(result != null && result.isValid()) ? result.getFiducialResults() : null;
			boolean tagVisible = tags != null && !tags.isEmpty();

			double drivePower = 0.0, turnPower = 0.0;
			String action;

			switch (mode) {
				case SCAN:
					turnPower = SCAN_POWER;
					// Record the closest distance seen for every tag during the spin.
					if (tagVisible) {
						for (LLResultTypes.FiducialResult f : tags) {
							double d = rangeInches(f);
							if (d <= 0) continue; // ignore bad/zero pose reads
							Double prev = closestByTag.get(f.getFiducialId());
							if (prev == null || d < prev) {
								closestByTag.put(f.getFiducialId(), d);
							}
						}
					}
					action = "SCAN spinning (" + closestByTag.size() + " tags found)";
					if (scanTimer.milliseconds() >= SCAN_DURATION_MS) {
						targetTagId = closestTag(closestByTag);
						if (targetTagId >= 0) {
							targetTagDistance = closestByTag.get(targetTagId);
							mode = Mode.TURN;
							note = "Closest tag = " + targetTagId + " @ " + String.format("%.1f in", targetTagDistance);
						} else {
							mode = Mode.MANUAL;
							note = "SCAN found no tags";
						}
					}
					break;

				case TURN: {
					LLResultTypes.FiducialResult target = findTag(tags, targetTagId);
					if (target != null) {
						double bearing = target.getTargetXDegrees();
						turnPower = clamp(bearing * TURN_GAIN, -MAX_TURN, MAX_TURN);
						if (Math.abs(bearing) <= BEARING_THRESHOLD) {
							mode = Mode.DRIVE;
							action = "TURN centered -> DRIVE";
						} else {
							action = String.format("TURN to tag %d (%.1f deg)", targetTagId, bearing);
						}
					} else {
						// Target not in view yet — keep spinning the same way to reacquire it.
						turnPower = SCAN_POWER;
						action = "TURN searching for tag " + targetTagId;
					}
					break;
				}

				case DRIVE:
					// Straight forward, no safety: keep going regardless of the tag until B.
					drivePower = FORWARD_POWER;
					action = "DRIVE straight (B to stop)";
					break;

				case MANUAL:
				default:
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

			// --- Status 2 & 3: live tag details ---
			if (tagVisible) {
				LLResultTypes.FiducialResult tag = tags.get(0);
				telemetry.addData("2) AprilTag", "IN RANGE (%d visible)", tags.size());
				telemetry.addData("   Nearest visible ID", tag.getFiducialId());
				telemetry.addData("3) Distance (in)", "%.1f", rangeInches(tag));
				telemetry.addData("   Bearing (deg)", "%.1f", tag.getTargetXDegrees());
			} else {
				telemetry.addData("2) AprilTag", "NONE IN RANGE");
				telemetry.addData("3) Distance (in)", "--");
			}

			// --- Auto debug indicators ---
			telemetry.addLine();
			telemetry.addData("MODE", mode + (mode == Mode.MANUAL ? " (A=start seq)" : " (B=stop)"));
			telemetry.addData("Action", action);
			if (mode == Mode.SCAN) {
				telemetry.addData("Scan progress", "%.0f / %d ms", scanTimer.milliseconds(), SCAN_DURATION_MS);
			}
			telemetry.addData("Tags found (id:dist)", closestByTag.isEmpty() ? "--" : closestByTag.toString());
			telemetry.addData("Target tag", targetTagId >= 0 ? targetTagId + " @ " + String.format("%.1f in", targetTagDistance) : "--");
			telemetry.addData("Cmd drive / turn", "%.2f / %.2f", drivePower, turnPower);
			if (!note.isEmpty()) telemetry.addData("Note", note);
			telemetry.update();
		}

		robot.limelight.stop();
	}

	/** Straight-ahead distance (in) from the camera to a tag; 0 if the pose is unavailable. */
	private double rangeInches(LLResultTypes.FiducialResult f) {
		return Math.abs(f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH).z);
	}

	/** Tag ID with the smallest recorded distance, or -1 if the map is empty. */
	private int closestTag(Map<Integer, Double> closestByTag) {
		int bestId = -1;
		double bestDist = Double.MAX_VALUE;
		for (Map.Entry<Integer, Double> e : closestByTag.entrySet()) {
			if (e.getValue() < bestDist) {
				bestDist = e.getValue();
				bestId = e.getKey();
			}
		}
		return bestId;
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
