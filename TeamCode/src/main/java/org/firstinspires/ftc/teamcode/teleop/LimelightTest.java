package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Collections;
import java.util.List;

/** Displays live Limelight 3A output for the pipeline selected on the Limelight. */
@TeleOp(name = "Test-Vision", group = "Test")
public class LimelightTest extends LinearOpMode {

	@Override
	public void runOpMode() {
		RobotHardware robot = new RobotHardware();
		robot.init(hardwareMap);

		robot.limelight.setPollRateHz(100);
		robot.limelight.pipelineSwitch(1);
		robot.limelight.start();

		telemetry.addLine("Test-Vision ready — press START");
		telemetry.update();
		waitForStart();

		while (opModeIsActive()) {
			boolean connected = false;
			LLStatus status = null;
			LLResult result = null;
			String communicationError = null;

			try {
				connected = robot.limelight.isConnected();
				status = robot.limelight.getStatus();
				// Poll once every loop so telemetry always reflects the latest frame.
				result = robot.limelight.getLatestResult();
			} catch (RuntimeException e) {
				// A transient network/device error must not end this diagnostic OpMode.
				communicationError = e.getClass().getSimpleName();
			}

			telemetry.addLine("=== LIMELIGHT STATUS ===");
			telemetry.addData("Connected", connected ? "Yes" : "No");
			if (status != null) {
				telemetry.addData("Selected pipeline", "%d (%s)",
						status.getPipelineIndex(), status.getPipelineType());
			} else {
				telemetry.addData("Selected pipeline", "Unavailable");
			}
			if (communicationError != null) {
				telemetry.addData("Communication", "Failed: %s", communicationError);
			}

			telemetry.addLine("=== FRAME RESULTS ===");
			boolean validResult = result != null && result.isValid();
			telemetry.addData("Valid result received", validResult ? "Yes" : "No");
			if (result == null) {
				telemetry.addData("Result", "No result available");
			} else if (!validResult) {
				telemetry.addData("Result", "Result received but is not valid");
			}

			if (validResult) {
				double captureLatency = result.getCaptureLatency();
				double targetingLatency = result.getTargetingLatency();
				telemetry.addData("Result pipeline", result.getPipelineIndex());
				telemetry.addData("Capture latency (ms)", "%.1f", captureLatency);
				telemetry.addData("Targeting latency (ms)", "%.1f", targetingLatency);
				telemetry.addData("Total latency (ms)", "%.1f", captureLatency + targetingLatency);

				// DetectorResult has no pose API in the installed FTC SDK; botpose is the available pose.
				Pose3D robotPose = result.getBotpose();
				if (robotPose != null) {
					telemetry.addData("Robot pose", robotPose);
				}

				List<LLResultTypes.DetectorResult> targets = result.getDetectorResults();
				if (targets == null) {
					targets = Collections.emptyList();
				}

				telemetry.addLine("=== DETECTED TARGETS ===");
				telemetry.addData("Number of targets", targets.size());
				if (targets.isEmpty()) {
					telemetry.addLine("No targets detected.");
				} else {
					for (int i = 0; i < targets.size(); i++) {
						LLResultTypes.DetectorResult target = targets.get(i);
						if (target == null) {
							telemetry.addData("Target " + (i + 1), "Unavailable");
							continue;
						}
						telemetry.addData("Target " + (i + 1), "%s (%.1f%%)",
								target.getClassName(), target.getConfidence());
						telemetry.addData("  tx / ty (deg)", "%.2f / %.2f",
								target.getTargetXDegrees(), target.getTargetYDegrees());
						telemetry.addData("  ta", "%.2f", target.getTargetArea());
					}
				}
			}

			telemetry.update();
		}

		robot.limelight.stop();
	}
}
