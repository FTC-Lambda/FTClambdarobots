package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.BallColor;
import org.firstinspires.ftc.teamcode.vision.BallTarget;
import org.firstinspires.ftc.teamcode.vision.BallVisionConfig;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;

/**
 * Diagnostic TeleOp for DECODE ball vision on pipeline 1.
 *
 * Controls:
 *   Sticks — manual drive
 *   A — switch to ball pipeline (1)
 *   B — switch to AprilTag pipeline (0)
 *   X / Y — prefer green / purple (or clear with bumpers)
 *   Back — toggle per-detection debug telemetry
 */
@TeleOp(name = "Ball Vision Test", group = "Test")
public class BallVisionTest extends LinearOpMode {

	@Override
	public void runOpMode() {
		RobotHardware robot = new RobotHardware();
		robot.init(hardwareMap);
		Drivetrain drivetrain = new Drivetrain(robot);

		LimelightVision vision = new LimelightVision(robot.limelight);
		// Select the pipeline before starting so the camera never boots on pipeline 0.
		vision.useBallDetectionPipeline();
		vision.start();
		vision.setDebugTelemetry(true);

		boolean debug = true;
		boolean prevA = false;
		boolean prevB = false;
		boolean prevX = false;
		boolean prevY = false;
		boolean prevBumper = false;
		boolean prevBack = false;

		telemetry.addLine("Ball Vision Test — A=balls B=AprilTag Back=debug");
		telemetry.update();
		waitForStart();

		while (opModeIsActive()) {
			// Edge-triggered: holding a button must not re-request a switch every loop.
			boolean bumper = gamepad1.left_bumper || gamepad1.right_bumper;
			if (gamepad1.a && !prevA) {
				vision.useBallDetectionPipeline();
			}
			if (gamepad1.b && !prevB) {
				vision.useAprilTagPipeline();
			}
			if (gamepad1.x && !prevX) {
				vision.setDesiredBallColor(BallColor.GREEN);
			}
			if (gamepad1.y && !prevY) {
				vision.setDesiredBallColor(BallColor.PURPLE);
			}
			if (bumper && !prevBumper) {
				vision.setDesiredBallColor(null);
			}
			if (gamepad1.back && !prevBack) {
				debug = !debug;
				vision.setDebugTelemetry(debug);
			}
			prevA = gamepad1.a;
			prevB = gamepad1.b;
			prevX = gamepad1.x;
			prevY = gamepad1.y;
			prevBumper = bumper;
			prevBack = gamepad1.back;

			vision.update();
			drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

			vision.addTelemetry(telemetry);
			BallTarget t = vision.getTarget();
			if (t.isValid() && vision.getRequestedPipeline() == BallVisionConfig.PIPELINE_BALL) {
				telemetry.addData("Aim hint", "tx=%.1f (steer with your own controller)",
						t.getHorizontalErrorDeg());
			}
			telemetry.update();
		}

		vision.stop();
	}
}
