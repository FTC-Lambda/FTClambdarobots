package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.BallAlignmentConfig;
import org.firstinspires.ftc.teamcode.control.BallAlignmentController;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.BallColor;
import org.firstinspires.ftc.teamcode.vision.BallGroup;
import org.firstinspires.ftc.teamcode.vision.BallTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;

/**
 * On-robot test for turning toward the selected ball target's horizontal center.
 *
 * <p>The vision subsystem selects and briefly persists the best group. {@link BallTarget}'s
 * confidence-and-area-weighted horizontal error is the sole control bearing; the selected
 * group's arithmetic mean is reported only as a diagnostic. This OpMode deliberately commands
 * rotation only: ball area is not a calibrated range, so it cannot safely provide AprilTag-style
 * approach or standoff control.</p>
 *
 * <p>Controls:</p>
 * <ul>
 *   <li>Sticks: manual mecanum drive while in MANUAL</li>
 *   <li>Cross (A): enter ALIGN and turn the wheels toward the selected group</li>
 *   <li>Circle (B): abort alignment and return to MANUAL</li>
 *   <li>Square (X) / Triangle (Y): prefer green / purple groups</li>
 *   <li>Either bumper (L1 / R1): clear the color preference</li>
 * </ul>
 */
@TeleOp(name = "Ball-Grouping-Aligning test", group = "Test")
public class BallGroupingAligningTest extends LinearOpMode {

	private enum Mode { MANUAL, ALIGN }

	@Override
	public void runOpMode() {
		RobotHardware robot = new RobotHardware();
		robot.init(hardwareMap);
		Drivetrain drivetrain = new Drivetrain(robot);

		LimelightVision vision = new LimelightVision(robot.limelight);
		vision.useBallDetectionPipeline();
		vision.start();

		BallAlignmentController alignment = new BallAlignmentController(BallAlignmentConfig.defaults());
		BallAlignmentController.Result alignmentResult =
				alignment.update(BallTarget.none(), System.currentTimeMillis(), 0.0);

		telemetry.addLine("Ball group alignment ready — press START");
		telemetry.addLine("X=ALIGN  O=MANUAL  Square=green  Triangle=purple  l1/r1=any color");
		telemetry.update();
		waitForStart();

		Mode mode = Mode.MANUAL;
		BallColor desiredColor = null;
		boolean prevA = false;
		boolean prevB = false;
		boolean prevX = false;
		boolean prevY = false;
		boolean prevBumper = false;
		ElapsedTime loopTimer = new ElapsedTime();

		try {
			while (opModeIsActive()) {
				double loopDt = loopTimer.seconds();
				loopTimer.reset();

				boolean bumper = gamepad1.left_bumper || gamepad1.right_bumper;
				if (gamepad1.x && !prevX) {
					desiredColor = BallColor.GREEN;
					vision.setDesiredBallColor(desiredColor);
					alignment.reset();
				} else if (gamepad1.y && !prevY) {
					desiredColor = BallColor.PURPLE;
					vision.setDesiredBallColor(desiredColor);
					alignment.reset();
				} else if (bumper && !prevBumper) {
					desiredColor = null;
					vision.setDesiredBallColor(null);
					alignment.reset();
				}

				if (gamepad1.b && !prevB) {
					mode = Mode.MANUAL;
					alignment.reset();
					alignmentResult = alignment.update(
							BallTarget.none(), System.currentTimeMillis(), loopDt);
				} else if (gamepad1.a && !prevA && mode == Mode.MANUAL) {
					mode = Mode.ALIGN;
					alignment.reset();
				}

				prevA = gamepad1.a;
				prevB = gamepad1.b;
				prevX = gamepad1.x;
				prevY = gamepad1.y;
				prevBumper = bumper;

				vision.update();
				BallTarget target = vision.getTarget();
				BallGroup selectedGroup = vision.getPersistedGroup().orElse(null);
				String action;

				if (mode == Mode.MANUAL) {
					drivetrain.drive(
							gamepad1.left_stick_y,
							gamepad1.left_stick_x,
							gamepad1.right_stick_x);
					action = "MANUAL — press Cross / A to align";
				} else {
					// The controller owns fresh/held target handling and commands an immediate zero
					// through its invalid-target path, so this test never searches blindly.
					alignmentResult = alignment.update(target, System.currentTimeMillis(), loopDt);
					drivetrain.driveRaw(0.0, 0.0, alignmentResult.getAppliedTurn());
					action = "ALIGN — " + alignmentResult.getAction();
				}

				telemetry.addData("MODE", mode + (mode == Mode.MANUAL
						? " (Cross/A=start)" : " (Circle/B=stop)"));
				telemetry.addData("Action", action);
				telemetry.addData("Color preference", desiredColor == null ? "ANY" : desiredColor);
				vision.addTelemetry(telemetry);
				telemetry.addData("Target state", target.isValid()
						? (target.isFresh() ? "FRESH" : "HELD " + target.getAgeMs() + " ms")
						: "NONE");
				telemetry.addData("Control bearing", target.isValid()
						? String.format("%.2f deg", target.getHorizontalErrorDeg()) : "--");
				if (selectedGroup != null) {
					telemetry.addData("Align center tx (mean)", "%.2f deg", selectedGroup.getAverageTxDeg());
				} else {
					telemetry.addData("Align center tx (mean)", "--");
				}
				telemetry.addData("Correction active", alignmentResult.isCorrectionActive());
				telemetry.addData("Turn PID (P/I/D)", "%.3f / %.3f / %.3f",
						alignmentResult.getP(), alignmentResult.getI(), alignmentResult.getD());
				telemetry.addData("Turn PID raw", "%.3f", alignmentResult.getRawPid());
				telemetry.addData("Turn command req/applied", "%.3f / %.3f",
						alignmentResult.getRequestedTurn(), alignmentResult.getAppliedTurn());
				telemetry.addData("Turn friction/slew", "%s / %s",
						alignmentResult.isStaticFrictionApplied() ? "YES" : "no",
						alignmentResult.isSlewLimited() ? "YES" : "no");
				telemetry.update();
			}
		} finally {
			drivetrain.driveRaw(0.0, 0.0, 0.0);
			vision.stop();
		}
	}
}
