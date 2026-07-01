package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.hardware.LimelightPipelines;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.List;

/**
 * Spins to find an AprilTag, centers on it, then drives up to a set distance.
 *
 * Pipeline 0 must be a fiducial (AprilTag) pipeline. This OpMode self-heals: at init it
 * converts pipeline 0 to fiducial from code if it isn't already (see LimelightPipelines),
 * so a Limelight that got reset to a color pipeline still works without the web UI.
 */
@Autonomous(name = "AprilTag Seek", group = "Auto")
public class AprilTagSeekAuto extends LinearOpMode {

	static final int PIPELINE_APRILTAG = 0; // Pipeline 0 must be a Fiducial/AprilTag pipeline

	// Printed tag edge length in mm (affects 3D distance/pose accuracy only). Measure yours.
	static final double FIDUCIAL_SIZE_MM = 165.1;

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

		robot.limelight.pipelineSwitch(PIPELINE_APRILTAG);
		robot.limelight.setPollRateHz(100); // ask the Limelight for data 100x/sec
		robot.limelight.start();

		// Self-heal: make sure pipeline 0 is actually a fiducial pipeline. If it was left
		// as a color pipeline ("pipe_color"), convert it in place from code (read-modify-
		// write over the Limelight REST API) so we don't blindly spin and time out.
		// Wait briefly for the device to be reachable first.
		long connectDeadline = System.currentTimeMillis() + 3000;
		while (!robot.limelight.isConnected() && System.currentTimeMillis() < connectDeadline) {
			sleep(50);
		}
		boolean pipelineOk = LimelightPipelines.ensureFiducial(
				robot.limelight, PIPELINE_APRILTAG,
				LimelightPipelines.FTC_FIDUCIAL_FAMILY, FIDUCIAL_SIZE_MM);
		if (pipelineOk) {
			robot.limelight.pipelineSwitch(PIPELINE_APRILTAG); // reload after a possible edit
			telemetry.addData("Status", "Pipeline 0 fiducial — ready");
		} else {
			telemetry.addData("Status", "PIPELINE NOT FIDUCIAL — Limelight unreachable?");
			telemetry.addLine("Run 'Limelight Pipeline Setup' with the hub connected.");
		}
		telemetry.update();
		waitForStart();

		if (!pipelineOk) {
			telemetry.addLine("Aborting: could not confirm a fiducial pipeline.");
			telemetry.update();
			robot.limelight.stop();
			return;
		}

		Phase phase = Phase.SPIN;
		int lockedTagId = -1;
		ElapsedTime timer = new ElapsedTime();

		// --- SPIN phase ---
		timer.reset();
		while (opModeIsActive() && phase == Phase.SPIN) {
			drivetrain.driveRaw(0, 0, SPIN_POWER);

			LLResult result = robot.limelight.getLatestResult();
			if (result != null && result.isValid()) {
				List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
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

		// --- CENTER phase ---
		timer.reset();
		while (opModeIsActive() && phase == Phase.CENTER) {
			LLResult result = robot.limelight.getLatestResult();
			LLResultTypes.FiducialResult locked = findTag(result, lockedTagId);

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
			LLResultTypes.FiducialResult locked = findTag(result, lockedTagId);

			if (locked != null) {
				double bearing = locked.getTargetXDegrees();
				double rangeIn = Math.abs(rangeInches(locked)); // forward distance to tag, inches

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

	private LLResultTypes.FiducialResult findTag(LLResult result, int tagId) {
		if (result == null || !result.isValid()) return null;
		for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
			if (f.getFiducialId() == tagId) return f;
		}
		return null;
	}

	// Forward distance from the camera to the locked tag, in inches. Uses the z axis of the
	// tag-pose-in-camera-space (straight-ahead distance), converted to inches. Requires the
	// 3D solve enabled (fiducial_skip3d=0), otherwise the pose is zeroed.
	private double rangeInches(LLResultTypes.FiducialResult f) {
		Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
		return p.z;
	}
}
