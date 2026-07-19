package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.util.VisionDeadband;
import org.firstinspires.ftc.teamcode.vision.BallColor;
import org.firstinspires.ftc.teamcode.vision.BallGroup;
import org.firstinspires.ftc.teamcode.vision.BallTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;

/**
 * On-robot test for turning toward the selected ball group's horizontal center.
 *
 * <p>The vision subsystem selects and briefly persists the best group. This test defines the
 * group's middle as the arithmetic mean of its members' horizontal angles; telemetry also
 * reports the confidence-and-area-weighted aimpoint exposed by {@link BallTarget}. This
 * OpMode deliberately commands rotation only: ball area is not a calibrated range, so it
 * cannot safely provide AprilTag-style approach or standoff control.</p>
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

	private static final double MAX_TURN_POWER = 0.35;
	private static final double TURN_MIN_POWER = 0.20;

	private enum Mode { MANUAL, ALIGN }

	@Override
	public void runOpMode() {
		RobotHardware robot = new RobotHardware();
		robot.init(hardwareMap);
		Drivetrain drivetrain = new Drivetrain(robot);

		LimelightVision vision = new LimelightVision(robot.limelight);
		vision.useBallDetectionPipeline();
		vision.start();

		PIDController turnPid = new PIDController(
				Constants.VISION_SEEK_TURN_KP,
				Constants.VISION_SEEK_TURN_KI,
				Constants.VISION_SEEK_TURN_KD,
				Constants.VISION_SEEK_TURN_INTEGRAL_LIMIT,
				Constants.VISION_SEEK_TURN_INTEGRAL_ZONE_DEG,
				Constants.VISION_TURN_DERIVATIVE_FILTER);
		SlewRateLimiter turnSlew = new SlewRateLimiter(Constants.VISION_TURN_SLEW_RATE);
		VisionDeadband deadband = new VisionDeadband();

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
		double requestedTurn = 0.0;
		double smoothedTurn = 0.0;
		ElapsedTime loopTimer = new ElapsedTime();
		ElapsedTime freshFrameTimer = new ElapsedTime();

		// Seed the limiter at stopped so the first ALIGN command ramps up from zero.
		turnSlew.calculate(0.0, 0.0);

		try {
			while (opModeIsActive()) {
				double loopDt = loopTimer.seconds();
				loopTimer.reset();

				boolean bumper = gamepad1.left_bumper || gamepad1.right_bumper;
				if (gamepad1.x && !prevX) {
					desiredColor = BallColor.GREEN;
					vision.setDesiredBallColor(desiredColor);
					requestedTurn = 0.0;
					turnPid.reset();
					deadband.clearState();
				} else if (gamepad1.y && !prevY) {
					desiredColor = BallColor.PURPLE;
					vision.setDesiredBallColor(desiredColor);
					requestedTurn = 0.0;
					turnPid.reset();
					deadband.clearState();
				} else if (bumper && !prevBumper) {
					desiredColor = null;
					vision.setDesiredBallColor(null);
					requestedTurn = 0.0;
					turnPid.reset();
					deadband.clearState();
				}

				if (gamepad1.b && !prevB) {
					mode = Mode.MANUAL;
					requestedTurn = 0.0;
					smoothedTurn = 0.0;
					turnPid.reset();
					turnSlew.reset();
					turnSlew.calculate(0.0, loopDt);
					deadband.clearState();
				} else if (gamepad1.a && !prevA && mode == Mode.MANUAL) {
					mode = Mode.ALIGN;
					requestedTurn = 0.0;
					smoothedTurn = 0.0;
					turnPid.reset();
					turnSlew.reset();
					turnSlew.calculate(0.0, loopDt);
					deadband.clearState();
					freshFrameTimer.reset();
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
				} else if (target.isValid() && selectedGroup != null) {
					double bearing = selectedGroup.getAverageTxDeg();

					// Neural frames can repeat between control loops. Update PID only for a fresh
					// measurement, then hold that request while persistence bridges the short gap.
					if (target.isFresh()) {
						double measurementDt = freshFrameTimer.seconds();
						freshFrameTimer.reset();
						double turnOut = turnPid.calculate(bearing, 0.0, measurementDt);
						if (deadband.shouldCorrectTurn(bearing)) {
							requestedTurn = clamp(turnOut, -MAX_TURN_POWER, MAX_TURN_POWER);
							if (Math.abs(requestedTurn) < TURN_MIN_POWER) {
								requestedTurn = Math.copySign(TURN_MIN_POWER, bearing);
							}
						} else {
							requestedTurn = 0.0;
						}
					}

					smoothedTurn = turnSlew.calculate(requestedTurn, loopDt);
					drivetrain.driveRaw(0.0, 0.0, smoothedTurn);
					action = deadband.isBearingCentered(bearing)
							? "ALIGN — group centered"
							: (target.isFresh() ? "ALIGN — correcting" : "ALIGN — holding last frame");
				} else {
					// Never search blindly in this test; stop until a group is visible again.
					requestedTurn = 0.0;
					smoothedTurn = 0.0;
					drivetrain.driveRaw(0.0, 0.0, 0.0);
					turnPid.reset();
					turnSlew.reset();
					turnSlew.calculate(0.0, loopDt);
					deadband.clearState();
					freshFrameTimer.reset();
					action = "ALIGN — waiting for a ball group";
				}

				telemetry.addData("MODE", mode + (mode == Mode.MANUAL
						? " (Cross/A=start)" : " (Circle/B=stop)"));
				telemetry.addData("Action", action);
				telemetry.addData("Color preference", desiredColor == null ? "ANY" : desiredColor);
				vision.addTelemetry(telemetry);
				telemetry.addData("Target state", target.isValid()
						? (target.isFresh() ? "FRESH" : "HELD " + target.getAgeMs() + " ms")
						: "NONE");
				if (target.isValid() && selectedGroup != null) {
					telemetry.addData("Align center tx (mean)", "%.2f deg", selectedGroup.getAverageTxDeg());
					telemetry.addData("Vision aim tx (weighted)", "%.2f deg", target.getHorizontalErrorDeg());
					telemetry.addData("Centered",
							deadband.isBearingCentered(selectedGroup.getAverageTxDeg()) ? "YES" : "no");
				} else {
					telemetry.addData("Align center tx (mean)", "--");
					telemetry.addData("Vision aim tx (weighted)", "--");
					telemetry.addData("Centered", "--");
				}
				telemetry.addData("Turn PID (P/I/D)", "%.3f / %.3f / %.3f",
						turnPid.getLastP(), turnPid.getLastI(), turnPid.getLastD());
				telemetry.addData("Turn command raw/smoothed", "%.3f / %.3f",
						requestedTurn, smoothedTurn);
				telemetry.update();
			}
		} finally {
			drivetrain.driveRaw(0.0, 0.0, 0.0);
			vision.stop();
		}
	}

	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
}
