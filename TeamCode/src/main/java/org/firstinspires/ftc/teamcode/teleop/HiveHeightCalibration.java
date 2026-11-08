package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.LimelightPipelines;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.vision.BallVisionConfig;
import org.firstinspires.ftc.teamcode.vision.hive.HiveCell;
import org.firstinspires.ftc.teamcode.vision.hive.HiveDetectionConfig;
import org.firstinspires.ftc.teamcode.vision.hive.HiveDetector;
import org.firstinspires.ftc.teamcode.vision.hive.HiveTagObservation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Driver Station tool to measure the CELL tag heights for each HIVE position and suggest the
 * {@code HiveDetectionConfig} threshold and hysteresis.
 *
 * <p>Controls: INIT: X = BLUE, B = RED (default RED). After START: sticks drive manually.
 * A = record the visible own-alliance CELL median height as a RAISED sample, Y = as a LOWERED
 * sample, X = clear samples. D-pad up/down = camera pitch +/-0.5 deg, D-pad left/right =
 * camera height -/+0.25 in (to match a tape-measured tag height).</p>
 *
 * <p>All values are runtime-only. Copy the camera mount into {@code Constants} and the
 * suggested threshold/hysteresis into {@code HiveDetectionConfig} defaults. Nothing moves
 * during INIT (G403).</p>
 */
@TeleOp(name = "Hive Height Calibration", group = "Vision")
public class HiveHeightCalibration extends LinearOpMode {

	private static final double PITCH_STEP_DEG = 0.5;
	private static final double HEIGHT_STEP_IN = 0.25;

	/** Running count/mean/min/max of recorded heights. */
	static final class SampleStats {
		int count;
		double sum;
		double min = Double.POSITIVE_INFINITY;
		double max = Double.NEGATIVE_INFINITY;

		void add(double v) {
			count++;
			sum += v;
			min = Math.min(min, v);
			max = Math.max(max, v);
		}

		void clear() {
			count = 0;
			sum = 0.0;
			min = Double.POSITIVE_INFINITY;
			max = Double.NEGATIVE_INFINITY;
		}

		double mean() {
			return count == 0 ? Double.NaN : sum / count;
		}

		String describe() {
			if (count == 0) {
				return "n=0";
			}
			return String.format("n=%d mean=%.1f min=%.1f max=%.1f", count, mean(), min, max);
		}
	}

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
			telemetry.addLine("Hive Height Calibration - X=BLUE  B=RED, then press START");
			telemetry.addData("Alliance", alliance);
			telemetry.addData("Drivetrain", drivetrain != null ? "found" : "not configured - vision only");
			telemetry.addLine("Nothing moves until START.");
			telemetry.update();
		}
		if (isStopRequested()) {
			return;
		}

		startLimelight(limelight);
		double cameraHeightIn = Constants.LIMELIGHT_HEIGHT_IN;
		double cameraPitchDeg = Constants.LIMELIGHT_PITCH_DEG;
		HiveDetector detector = new HiveDetector(limelight, alliance,
				HiveDetectionConfig.defaults(), cameraHeightIn, cameraPitchDeg);

		SampleStats raised = new SampleStats();
		SampleStats lowered = new SampleStats();
		String lastAction = "none";

		boolean prevA = false;
		boolean prevY = false;
		boolean prevX = false;
		boolean prevUp = false;
		boolean prevDown = false;
		boolean prevLeft = false;
		boolean prevRight = false;

		try {
			while (opModeIsActive()) {
				detector.update();

				boolean mountChanged = false;
				if (gamepad1.dpad_up && !prevUp) {
					cameraPitchDeg += PITCH_STEP_DEG;
					mountChanged = true;
				}
				if (gamepad1.dpad_down && !prevDown) {
					cameraPitchDeg -= PITCH_STEP_DEG;
					mountChanged = true;
				}
				if (gamepad1.dpad_right && !prevRight) {
					cameraHeightIn += HEIGHT_STEP_IN;
					mountChanged = true;
				}
				if (gamepad1.dpad_left && !prevLeft) {
					cameraHeightIn -= HEIGHT_STEP_IN;
					mountChanged = true;
				}
				if (mountChanged) {
					detector.setCameraMount(cameraHeightIn, cameraPitchDeg);
				}

				// Per-CELL median of OWN-alliance tags from the last accepted frame.
				double[] medians = new double[HiveCell.values().length];
				int[] counts = new int[medians.length];
				for (HiveCell cell : HiveCell.values()) {
					List<Double> heights = new ArrayList<>();
					for (HiveTagObservation o : detector.getLastObservations()) {
						if (o.getAlliance() == alliance && o.getCell() == cell) {
							heights.add(o.getHeightIn());
						}
					}
					medians[cell.ordinal()] = median(heights);
					counts[cell.ordinal()] = heights.size();
				}

				if (gamepad1.a && !prevA) {
					lastAction = recordMessage("A", "RAISED", record(raised, medians));
				}
				if (gamepad1.y && !prevY) {
					lastAction = recordMessage("Y", "LOWERED", record(lowered, medians));
				}
				if (gamepad1.x && !prevX) {
					raised.clear();
					lowered.clear();
					lastAction = "X: samples cleared";
				}
				prevA = gamepad1.a;
				prevY = gamepad1.y;
				prevX = gamepad1.x;
				prevUp = gamepad1.dpad_up;
				prevDown = gamepad1.dpad_down;
				prevLeft = gamepad1.dpad_left;
				prevRight = gamepad1.dpad_right;

				if (drivetrain != null) drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

				telemetry.addLine("Hive Height Calibration (" + alliance + ")");
				telemetry.addLine("A=RAISED sample  Y=LOWERED sample  X=clear  sticks=drive");
				for (HiveCell cell : HiveCell.values()) {
					double m = medians[cell.ordinal()];
					telemetry.addData(cell + " cell median", "%s in  (tags=%d)",
							Double.isNaN(m) ? "--" : String.format("%.1f", m), counts[cell.ordinal()]);
				}
				telemetry.addData("Last action", lastAction);
				telemetry.addData("RAISED samples", raised.describe());
				telemetry.addData("LOWERED samples", lowered.describe());
				addSuggestion(raised, lowered);
				telemetry.addData("No-3D tags (this frame)", detector.getLastNo3dCount());
				telemetry.addLine("D-pad up/down = pitch +/-0.5 deg; left/right = height -/+0.25 in");
				telemetry.addData("Camera mount (runtime)", "height=%.2f in  pitch=%.1f deg",
						cameraHeightIn, cameraPitchDeg);
				telemetry.addLine("Copy mount into Constants.LIMELIGHT_HEIGHT_IN / _PITCH_DEG.");
				telemetry.update();
			}
		} finally {
			if (drivetrain != null) drivetrain.driveRaw(0.0, 0.0, 0.0);
			limelight.stop();
		}
	}

	/**
	 * Records the median of the one visible own-alliance CELL. Returns 1 if recorded, 0 if no
	 * CELL is visible, and -1 if both are visible. With both in view, one is raised and one is
	 * lowered, so labelling both with the same button would corrupt the samples.
	 */
	private static int record(SampleStats stats, double[] medians) {
		int visible = 0;
		double value = Double.NaN;
		for (double m : medians) {
			if (!Double.isNaN(m)) {
				visible++;
				value = m;
			}
		}
		if (visible != 1) {
			return visible == 0 ? 0 : -1;
		}
		stats.add(value);
		return 1;
	}

	private static String recordMessage(String button, String label, int result) {
		if (result == 0) {
			return button + ": no own CELL visible";
		}
		if (result < 0) {
			return button + ": both CELLs visible - aim at just one";
		}
		return button + ": " + label + " sample recorded";
	}

	private void addSuggestion(SampleStats raised, SampleStats lowered) {
		if (raised.count == 0 || lowered.count == 0) {
			telemetry.addLine("Record both RAISED and LOWERED samples for a suggestion.");
			return;
		}
		if (raised.min <= lowered.max) {
			telemetry.addLine("WARNING: RAISED min <= LOWERED max - overlap - check camera "
					+ "height/pitch and tag size");
			return;
		}
		double threshold = (lowered.max + raised.min) / 2.0;
		double hysteresis = Math.max(0.5, (raised.min - lowered.max) / 4.0);
		telemetry.addData("Suggested raisedThresholdIn", "%.1f", threshold);
		telemetry.addData("Suggested hysteresisIn", "%.2f", hysteresis);
	}

	private static double median(List<Double> values) {
		if (values.isEmpty()) {
			return Double.NaN;
		}
		List<Double> sorted = new ArrayList<>(values);
		Collections.sort(sorted);
		int n = sorted.size();
		return n % 2 == 1 ? sorted.get(n / 2) : (sorted.get(n / 2 - 1) + sorted.get(n / 2)) / 2.0;
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
