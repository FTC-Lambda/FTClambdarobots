package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.LimelightPipelines;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * One-time setup OpMode that reconfigures Limelight pipeline 0 into an AprilTag
 * (fiducial) pipeline entirely from robot code — no web UI needed.
 *
 * WHY THIS EXISTS: the Limelight web UI (port 5801) is only reachable from a device
 * on the Limelight's USB-Ethernet subnet. A laptop on the robot WiFi has no route to
 * it, but the Control Hub is directly on that subnet, so robot code CAN talk to the
 * Limelight's REST API. The FTC SDK exposes that as getPipelineAtIndex()/updatePipeline().
 *
 * WHAT IT DOES (read-modify-write, so nothing else in the pipeline is clobbered):
 *   1. Downloads pipeline 0's current JSON from the Limelight.
 *   2. Sets pipeline_type = "pipe_fiducial", fiducial_type = "aprilClassic36h11" (FTC),
 *      and fiducial_size to your printed tag size.
 *   3. Writes it back with flush=true so it persists across power cycles.
 *
 * HOW TO USE:
 *   - Run this OpMode once. It shows the current pipeline_type on init.
 *   - Press START, then hold gamepad1.A to apply the fiducial config.
 *   - Confirm it reports "pipe_fiducial", then stop. Run "Limelight Test" to verify tags.
 *   - You only need to do this once per Limelight (the setting is saved on the device).
 */
@TeleOp(name = "Limelight Pipeline Setup", group = "Setup")
public class LimelightPipelineSetup extends LinearOpMode {

	static final int PIPELINE_INDEX = 0;

	// Physical size of your printed tag = the black square's edge length, IN MILLIMETERS.
	// This only affects 3D pose/distance accuracy — detection (tx/ty) works even if it's
	// a little off. MEASURE YOUR TAG and set this. (165.1 mm is the FRC field-tag size.)
	static final double FIDUCIAL_SIZE_MM = 165.1;

	private RobotHardware robot;

	@Override
	public void runOpMode() {
		robot = new RobotHardware();
		robot.init(hardwareMap);

		// Make slot 0 active and start polling so isConnected() becomes meaningful.
		robot.limelight.pipelineSwitch(PIPELINE_INDEX);
		robot.limelight.start();

		telemetry.addLine("Limelight Pipeline Setup");
		telemetry.addLine("Waiting for Limelight to connect...");
		telemetry.update();

		// Wait for the device to be reachable before we try HTTP reads/writes.
		long deadline = System.currentTimeMillis() + 5000;
		while (!isStopRequested() && !robot.limelight.isConnected()
				&& System.currentTimeMillis() < deadline) {
			sleep(50);
		}

		String currentType = readPipelineType();
		telemetry.addLine("Limelight Pipeline Setup");
		telemetry.addData("Connected", robot.limelight.isConnected());
		telemetry.addData("Pipeline 0 type (now)", currentType);
		if ("pipe_fiducial".equals(currentType)) {
			telemetry.addLine("Already fiducial — nothing to do. You can stop.");
		} else {
			telemetry.addLine("Press START, then hold A to convert pipeline 0 to fiducial.");
		}
		telemetry.update();

		waitForStart();

		boolean applied = false;
		while (opModeIsActive()) {
			if (!applied && gamepad1.a) {
				applied = true;
				boolean ok = applyFiducialConfig();
				telemetry.addLine(ok ? "Applied. Re-reading to verify..." : "updatePipeline() returned FALSE");
				telemetry.update();
				sleep(500); // give the device a moment to persist + reload
			}

			LLStatus status = robot.limelight.getStatus();
			telemetry.addData("Live pipeline type", status != null ? status.getPipelineType() : "??");
			telemetry.addData("Stored pipeline_type", readPipelineType());
			if (!applied) {
				telemetry.addLine("Hold gamepad1.A to apply fiducial config to pipeline 0");
			} else {
				telemetry.addLine("Done. If it reads pipe_fiducial / fiducial, stop and run Limelight Test.");
			}
			telemetry.update();
		}

		robot.limelight.stop();
	}

	/** Reads pipeline 0's JSON from the device and returns its pipeline_type field. */
	private String readPipelineType() {
		String type = LimelightPipelines.pipelineType(robot.limelight, PIPELINE_INDEX);
		return type != null ? type : "(no response)";
	}

	/** Read-modify-write conversion of pipeline 0 to fiducial (preserves all other fields). */
	private boolean applyFiducialConfig() {
		return LimelightPipelines.ensureFiducial(
				robot.limelight, PIPELINE_INDEX,
				LimelightPipelines.FTC_FIDUCIAL_FAMILY, FIDUCIAL_SIZE_MM);
	}
}
