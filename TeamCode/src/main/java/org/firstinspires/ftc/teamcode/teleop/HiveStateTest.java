package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.LimelightPipelines;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.vision.BallVisionConfig;
import org.firstinspires.ftc.teamcode.vision.hive.HiveAccuracyTally;
import org.firstinspires.ftc.teamcode.vision.hive.HiveCell;
import org.firstinspires.ftc.teamcode.vision.hive.HiveDetector;
import org.firstinspires.ftc.teamcode.vision.hive.HiveLabels;
import org.firstinspires.ftc.teamcode.vision.hive.HiveState;

/**
 * Diagnostic TeleOp for HIVE state detection: which CELL of OUR alliance's HIVE faces up.
 *
 * <p>Controls: during INIT, X = BLUE and B = RED alliance (default RED). After START the sticks
 * drive manually; telemetry shows the confirmed HIVE state, per-CELL heights and diagnostics,
 * and the gamepad rumbles whenever the confirmed state changes.</p>
 *
 * <p>Accuracy test (docs/game/HIVE_NAMING.md): tell it the REAL HIVE state with D-pad up =
 * FAR UP, D-pad down = AUDIENCE UP, D-pad left = stop scoring. A = restart the tally at a new
 * test spot. Telemetry then scores every camera frame against the truth.</p>
 *
 * <p>Nothing moves during INIT (G403): the drivetrain is only commanded after START.</p>
 */
@TeleOp(name = "Hive State Test", group = "Vision")
public class HiveStateTest extends LinearOpMode {

	@Override
	public void runOpMode() {
		Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
		// Optional: null when the drive motors are not configured (bench test with only the Limelight).
		Drivetrain drivetrain = Drivetrain.tryCreate(hardwareMap);

		Alliance alliance = Alliance.RED;
		while (!isStarted() && !isStopRequested()) {
			if (gamepad1.x) {
				alliance = Alliance.BLUE;
			} else if (gamepad1.b) {
				alliance = Alliance.RED;
			}
			telemetry.addLine("Hive State Test - X=BLUE  B=RED, then press START");
			telemetry.addData("Alliance", alliance);
			telemetry.addData("Drivetrain", drivetrain != null ? "found" : "not configured - vision only");
			telemetry.addLine("Nothing moves until START.");
			telemetry.update();
		}
		if (isStopRequested()) {
			return;
		}

		startLimelight(limelight);
		HiveDetector detector = new HiveDetector(limelight, alliance);
		HiveState lastState = detector.getState();
		HiveAccuracyTally tally = new HiveAccuracyTally();
		int lastFrameCount = detector.getFramesProcessed();
		boolean prevUp = false;
		boolean prevDown = false;
		boolean prevLeft = false;
		boolean prevA = false;

		try {
			while (opModeIsActive()) {
				detector.update();

				// Ground truth entry (edge-triggered).
				if (gamepad1.dpad_up && !prevUp) {
					tally.setTruth(HiveState.FAR_CELL_UP);
				}
				if (gamepad1.dpad_down && !prevDown) {
					tally.setTruth(HiveState.AUDIENCE_CELL_UP);
				}
				if (gamepad1.dpad_left && !prevLeft) {
					tally.setTruth(HiveState.UNKNOWN);
				}
				if (gamepad1.a && !prevA) {
					tally.reset();
				}
				prevUp = gamepad1.dpad_up;
				prevDown = gamepad1.dpad_down;
				prevLeft = gamepad1.dpad_left;
				prevA = gamepad1.a;

				// Score once per NEW camera frame, not once per loop.
				int frameCount = detector.getFramesProcessed();
				if (frameCount != lastFrameCount) {
					lastFrameCount = frameCount;
					int ownTags = detector.getEstimator().getCellTagCount(HiveCell.FAR)
							+ detector.getEstimator().getCellTagCount(HiveCell.AUDIENCE);
					tally.record(detector.getEstimator().getLastFrameVote(), detector.getState(), ownTags);
				}

				HiveState state = detector.getState();
				if (state != lastState) {
					gamepad1.rumble(300);
					lastState = state;
				}

				if (drivetrain != null) drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

				telemetry.addLine("Hive State Test (sticks = drive)");
				detector.addTelemetry(telemetry);
				telemetry.addLine("--- Accuracy (D-pad up=FAR UP, down=AUDIENCE UP, left=off; A=new spot) ---");
				telemetry.addData("Truth", HiveLabels.stateName(tally.getTruth()));
				if (tally.getTruth() != HiveState.UNKNOWN) {
					telemetry.addData("Per frame", tally.frameSummary());
					telemetry.addData("Confirmed", tally.confirmedSummary());
				}
				telemetry.update();
			}
		} finally {
			if (drivetrain != null) drivetrain.driveRaw(0.0, 0.0, 0.0);
			limelight.stop();
		}
	}

	/** Starts the Limelight on the AprilTag pipeline and repairs it to fiducial if needed. */
	private void startLimelight(Limelight3A limelight) {
		limelight.setPollRateHz(BallVisionConfig.POLL_RATE_HZ);
		limelight.pipelineSwitch(BallVisionConfig.PIPELINE_APRILTAG);
		limelight.start();
		long deadline = System.currentTimeMillis() + 3000;
		while (opModeIsActive() && !limelight.isConnected()
				&& System.currentTimeMillis() < deadline) {
			sleep(50);
		}
		boolean ok = LimelightPipelines.ensureFiducial(limelight, BallVisionConfig.PIPELINE_APRILTAG,
				LimelightPipelines.FTC_FIDUCIAL_FAMILY, Constants.APRILTAG_SIZE_MM);
		if (ok) {
			limelight.pipelineSwitch(BallVisionConfig.PIPELINE_APRILTAG); // reload after a possible edit
		} else {
			telemetry.addLine("PIPELINE NOT FIDUCIAL - run Limelight Pipeline Setup");
			telemetry.update();
		}
	}
}
