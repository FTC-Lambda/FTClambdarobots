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
 */
@TeleOp(name = "Ball Vision Test", group = "Test")
public class BallVisionTest extends LinearOpMode {

	@Override
	public void runOpMode() {
		RobotHardware robot = new RobotHardware();
		robot.init(hardwareMap);
		Drivetrain drivetrain = new Drivetrain(robot);

		LimelightVision vision = new LimelightVision(robot.limelight);
		vision.start();
		vision.useBallDetectionPipeline();

		telemetry.addLine("Ball Vision Test — A=balls B=AprilTag");
		telemetry.update();
		waitForStart();

		while (opModeIsActive()) {
			if (gamepad1.a) {
				vision.useBallDetectionPipeline();
			}
			if (gamepad1.b) {
				vision.useAprilTagPipeline();
			}
			if (gamepad1.x) {
				vision.setDesiredBallColor(BallColor.GREEN);
			}
			if (gamepad1.y) {
				vision.setDesiredBallColor(BallColor.PURPLE);
			}
			if (gamepad1.left_bumper || gamepad1.right_bumper) {
				vision.setDesiredBallColor(null);
			}

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
