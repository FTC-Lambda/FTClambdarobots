package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.BallAlignmentConfig;
import org.firstinspires.ftc.teamcode.control.BallAlignmentController;
import org.firstinspires.ftc.teamcode.control.BallAlignmentTuningModel;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.BallColor;
import org.firstinspires.ftc.teamcode.vision.BallGroup;
import org.firstinspires.ftc.teamcode.vision.BallTarget;
import org.firstinspires.ftc.teamcode.vision.BallTargetingConfig;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;

/**
 * Driver Station tuner for the shared ball-target selection and turn-alignment controllers.
 *
 * <p>All changes are session-only. Copy validated values into the corresponding defaults before
 * relying on them in another OpMode run.</p>
 */
@TeleOp(name = "Ball-Group Alignment Tuner", group = "Test")
public class BallGroupAlignmentTuner extends LinearOpMode {

	private enum Mode { MANUAL, ALIGN }

	@Override
	public void runOpMode() {
		RobotHardware robot = new RobotHardware();
		robot.init(hardwareMap);
		Drivetrain drivetrain = new Drivetrain(robot);

		LimelightVision vision = new LimelightVision(robot.limelight);
		vision.useBallDetectionPipeline();
		vision.start();

		BallAlignmentTuningModel tuning = new BallAlignmentTuningModel();
		BallAlignmentController alignment =
				new BallAlignmentController(tuning.getAlignmentConfig());
		BallAlignmentController.Result alignmentResult =
				alignment.update(BallTarget.none(), System.currentTimeMillis(), 0.0);

		telemetry.addLine("Ball-group alignment tuner ready - press START");
		telemetry.addLine("A=ALIGN  B=MANUAL  X=green  Y=purple  bumpers=any");
		telemetry.addLine("D-pad up/down=select  left/right=adjust  LS button=coarse");
		telemetry.addLine("Values are session-only; copy validated values into defaults.");
		telemetry.update();
		waitForStart();

		Mode mode = Mode.MANUAL;
		BallColor desiredColor = null;
		boolean prevA = false;
		boolean prevB = false;
		boolean prevX = false;
		boolean prevY = false;
		boolean prevLeftBumper = false;
		boolean prevRightBumper = false;
		boolean prevDpadUp = false;
		boolean prevDpadDown = false;
		boolean prevDpadLeft = false;
		boolean prevDpadRight = false;
		ElapsedTime loopTimer = new ElapsedTime();

		try {
			while (opModeIsActive()) {
				double loopDt = loopTimer.seconds();
				loopTimer.reset();
				long nowMs = System.currentTimeMillis();
				boolean forceZeroThisLoop = false;

				if (gamepad1.x && !prevX) {
					desiredColor = BallColor.GREEN;
					vision.setDesiredBallColor(desiredColor);
					alignment.reset();
					drivetrain.driveRaw(0.0, 0.0, 0.0);
					forceZeroThisLoop = true;
				} else if (gamepad1.y && !prevY) {
					desiredColor = BallColor.PURPLE;
					vision.setDesiredBallColor(desiredColor);
					alignment.reset();
					drivetrain.driveRaw(0.0, 0.0, 0.0);
					forceZeroThisLoop = true;
				} else if ((gamepad1.left_bumper && !prevLeftBumper)
						|| (gamepad1.right_bumper && !prevRightBumper)) {
					desiredColor = null;
					vision.setDesiredBallColor(null);
					alignment.reset();
					drivetrain.driveRaw(0.0, 0.0, 0.0);
					forceZeroThisLoop = true;
				}

				if (gamepad1.b && !prevB) {
					mode = Mode.MANUAL;
					alignment.reset();
					drivetrain.driveRaw(0.0, 0.0, 0.0);
					forceZeroThisLoop = true;
				} else if (gamepad1.a && !prevA && mode == Mode.MANUAL) {
					mode = Mode.ALIGN;
					alignment.reset();
				}

				boolean coarse = gamepad1.left_stick_button;
				if (gamepad1.dpad_up && !prevDpadUp) {
					tuning.selectPrevious();
				}
				if (gamepad1.dpad_down && !prevDpadDown) {
					tuning.selectNext();
				}
				if (gamepad1.dpad_left && !prevDpadLeft) {
					applyChange(
							tuning.adjust(-1, coarse), tuning, alignment, vision, drivetrain);
					forceZeroThisLoop = true;
				}
				if (gamepad1.dpad_right && !prevDpadRight) {
					applyChange(
							tuning.adjust(1, coarse), tuning, alignment, vision, drivetrain);
					forceZeroThisLoop = true;
				}

				prevA = gamepad1.a;
				prevB = gamepad1.b;
				prevX = gamepad1.x;
				prevY = gamepad1.y;
				prevLeftBumper = gamepad1.left_bumper;
				prevRightBumper = gamepad1.right_bumper;
				prevDpadUp = gamepad1.dpad_up;
				prevDpadDown = gamepad1.dpad_down;
				prevDpadLeft = gamepad1.dpad_left;
				prevDpadRight = gamepad1.dpad_right;

				vision.update();
				BallTarget target = vision.getTarget();
				BallGroup lockedGroup = vision.getPersistedGroup().orElse(null);
				BallGroup pendingGroup = vision.getPendingBallGroup().orElse(null);

				if (forceZeroThisLoop) {
					alignmentResult =
							alignment.update(BallTarget.none(), nowMs, loopDt);
				} else if (mode == Mode.MANUAL) {
					alignmentResult =
							alignment.update(BallTarget.none(), nowMs, loopDt);
					drivetrain.drive(
							gamepad1.left_stick_y,
							gamepad1.left_stick_x,
							gamepad1.right_stick_x);
				} else {
					BallAlignmentController.Result result =
							alignment.update(vision.getTarget(), System.currentTimeMillis(), loopDt);
					drivetrain.driveRaw(0.0, 0.0, result.getAppliedTurn());
					alignmentResult = result;
				}

				addTuningTelemetry(tuning, coarse);
				telemetry.addData("MODE", mode + (mode == Mode.MANUAL
						? " (A=start)" : " (B=stop)"));
				telemetry.addData("Requested color",
						desiredColor == null ? "ANY" : desiredColor);
				telemetry.addData("Target state", targetState(target));
				telemetry.addData("Target age", target.isValid()
						? target.getAgeMs() + " ms" : "--");
				telemetry.addData("Lock/pending sizes", "%d / %d",
						lockedGroup == null ? 0 : lockedGroup.getSize(),
						pendingGroup == null ? 0 : pendingGroup.getSize());
				telemetry.addData("Pending confirmation", "%d / %d",
						vision.getPendingBallConfirmationFrames(),
						tuning.getTargetingConfig().getSwitchConfirmFrames());
				telemetry.addData("Bearing raw / filtered", "%s / %s",
						lockedGroup == null ? "--" : formatDegrees(vision.getRawLockedTxDeg()),
						target.isValid() ? formatDegrees(target.getHorizontalErrorDeg()) : "--");
				telemetry.addData("Bearing diagnostic mean",
						lockedGroup == null ? "--" : formatDegrees(lockedGroup.getAverageTxDeg()));
				telemetry.addData("Controller action", alignmentResult.getAction());
				telemetry.addData("Correction active", alignmentResult.isCorrectionActive());
				telemetry.addData("Turn PID (P/I/D)", "%.4f / %.4f / %.4f",
						alignmentResult.getP(), alignmentResult.getI(), alignmentResult.getD());
				telemetry.addData("Turn PID raw", "%.4f", alignmentResult.getRawPid());
				telemetry.addData("Turn requested/applied", "%.4f / %.4f",
						alignmentResult.getRequestedTurn(), alignmentResult.getAppliedTurn());
				telemetry.addData("Static friction", yesNo(
						alignmentResult.isStaticFrictionApplied()));
				telemetry.addData("Slew limited", yesNo(alignmentResult.isSlewLimited()));
				telemetry.addData("Command expired", yesNo(
						alignmentResult.getAction()
								== BallAlignmentController.Action.COMMAND_EXPIRED));
				telemetry.addData("Reversal guard", yesNo(
						alignmentResult.getAction()
								== BallAlignmentController.Action.REVERSAL_GUARD));
				vision.addTelemetry(telemetry);
				telemetry.update();
			}
		} finally {
			drivetrain.driveRaw(0.0, 0.0, 0.0);
			alignment.reset();
			vision.stop();
		}
	}

	private static void applyChange(
			BallAlignmentTuningModel.ChangeDomain domain,
			BallAlignmentTuningModel tuning,
			BallAlignmentController alignment,
			LimelightVision vision,
			Drivetrain drivetrain) {
		if (domain == BallAlignmentTuningModel.ChangeDomain.ALIGNMENT) {
			alignment.setConfig(tuning.getAlignmentConfig());
		} else {
			vision.setBallTargetingConfig(tuning.getTargetingConfig());
			alignment.reset();
		}
		drivetrain.driveRaw(0.0, 0.0, 0.0);
	}

	private void addTuningTelemetry(BallAlignmentTuningModel tuning, boolean coarse) {
		BallAlignmentConfig currentAlignment = tuning.getAlignmentConfig();
		BallTargetingConfig currentTargeting = tuning.getTargetingConfig();
		BallAlignmentConfig defaultAlignment = BallAlignmentConfig.defaults();
		BallTargetingConfig defaultTargeting = BallTargetingConfig.defaults();
		double selectedIncrement = coarse
				? tuning.getCoarseIncrement() : tuning.getFineIncrement();

		telemetry.addData("Selected parameter",
				tuning.getSelectedParameter().getDisplayName());
		telemetry.addData("Increment", "%s (%.4f)",
				coarse ? "COARSE" : "fine", selectedIncrement);
		telemetry.addData("Selected current/default", "%.4f / %.4f",
				tuning.getSelectedValue(), tuning.getSelectedDefaultValue());
		telemetry.addLine("Runtime / default (session-only)");
		telemetry.addData("turn kP", "%.4f / %.4f",
				currentAlignment.getTurnKp(), defaultAlignment.getTurnKp());
		telemetry.addData("turn kI", "%.4f / %.4f",
				currentAlignment.getTurnKi(), defaultAlignment.getTurnKi());
		telemetry.addData("turn kD", "%.4f / %.4f",
				currentAlignment.getTurnKd(), defaultAlignment.getTurnKd());
		telemetry.addData("derivative filter", "%.4f / %.4f",
				currentAlignment.getDerivativeFilter(), defaultAlignment.getDerivativeFilter());
		telemetry.addData("start deadband deg", "%.3f / %.3f",
				currentAlignment.getStartCorrectionDeg(),
				defaultAlignment.getStartCorrectionDeg());
		telemetry.addData("stop deadband deg", "%.3f / %.3f",
				currentAlignment.getStopCorrectionDeg(),
				defaultAlignment.getStopCorrectionDeg());
		telemetry.addData("max turn power", "%.3f / %.3f",
				currentAlignment.getMaxTurnPower(), defaultAlignment.getMaxTurnPower());
		telemetry.addData("static friction", "%.3f / %.3f",
				currentAlignment.getStaticFrictionPower(),
				defaultAlignment.getStaticFrictionPower());
		telemetry.addData("turn slew rate", "%.3f / %.3f",
				currentAlignment.getTurnSlewRate(), defaultAlignment.getTurnSlewRate());
		telemetry.addData("command hold ms", "%d / %d",
				currentAlignment.getCommandHoldMs(), defaultAlignment.getCommandHoldMs());
		telemetry.addData("switch confirm frames", "%d / %d",
				currentTargeting.getSwitchConfirmFrames(),
				defaultTargeting.getSwitchConfirmFrames());
		telemetry.addData("aim filter gain", "%.3f / %.3f",
				currentTargeting.getAimFilterGain(), defaultTargeting.getAimFilterGain());
	}

	private static String targetState(BallTarget target) {
		if (!target.isValid()) {
			return "INVALID";
		}
		return target.isFresh() ? "FRESH" : "HELD";
	}

	private static String formatDegrees(double value) {
		return String.format("%.2f deg", value);
	}

	private static String yesNo(boolean value) {
		return value ? "YES" : "no";
	}
}
