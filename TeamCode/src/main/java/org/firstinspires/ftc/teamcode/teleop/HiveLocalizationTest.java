package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.hardware.LimelightPipelines;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.vision.BallVisionConfig;
import org.firstinspires.ftc.teamcode.vision.hive.FieldPoseEstimate;
import org.firstinspires.ftc.teamcode.vision.hive.HiveGeometry;
import org.firstinspires.ftc.teamcode.vision.hive.HiveLocalizer;
import org.firstinspires.ftc.teamcode.vision.hive.HiveTagMap;
import org.firstinspires.ftc.teamcode.vision.hive.TagSighting;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Locale;

/**
 * Field localization from the HIVE AprilTags (docs/game/HIVE_LOCALIZATION.md).
 *
 * <p>Field frame: inches from the A1 corner, +x toward column F, +y toward the far wall,
 * heading counter-clockwise from +x. The pose is the robot center (or the camera itself
 * when LIMELIGHT_FORWARD_IN / LEFT_IN are 0, e.g. a camera on a stand).</p>
 *
 * <p>Modes: TAGS ONLY fits x, y and heading from 2+ tags. If a goBILDA Pinpoint named
 * "pinpoint" is configured, press A while TAGS ONLY is steady to align its gyro to the field;
 * after that the gyro supplies heading and even a single tag gives x/y.</p>
 *
 * <p>Controls: A = align Pinpoint heading to the tags, B = drop the alignment. Sticks drive
 * if the drivetrain is configured. Runs with only the Limelight connected. Nothing moves in INIT.</p>
 */
@TeleOp(name = "Hive Localization Test", group = "Vision")
public class HiveLocalizationTest extends LinearOpMode {

	/** Fit-quality gate for samples used to align the gyro. */
	private static final double ALIGN_MAX_RMS_IN = 2.0;
	private static final int ALIGN_MIN_SAMPLES = 5;
	private static final int ALIGN_WINDOW = 15;
	private static final int JITTER_WINDOW = 20;

	@Override
	public void runOpMode() {
		Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
		Drivetrain drivetrain = Drivetrain.tryCreate(hardwareMap);
		GoBildaPinpointDriver pinpoint = null;
		try {
			pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
		} catch (RuntimeException e) {
			pinpoint = null;
		}

		telemetry.addLine("Hive Localization Test - keep the robot still during INIT, then START.");
		telemetry.addData("Drivetrain", drivetrain != null ? "found" : "not configured - vision only");
		telemetry.addData("Pinpoint", pinpoint != null ? "found (gyro heading available)" : "not configured - tags only");
		telemetry.update();
		if (pinpoint != null) {
			pinpoint.resetPosAndIMU(); // recalibrates the gyro; the robot must be still
		}
		waitForStart();
		if (isStopRequested()) {
			return;
		}

		startLimelight(limelight);
		HiveLocalizer localizer = HiveLocalizer.defaults();

		double headingOffsetDeg = Double.NaN; // field heading = gyro heading + offset
		Deque<Double> alignSamples = new ArrayDeque<>();
		Deque<double[]> jitter = new ArrayDeque<>();
		double lastFrameTs = Double.NaN;
		FieldPoseEstimate tagsOnly = null;
		FieldPoseEstimate withGyro = null;
		boolean prevA = false;
		boolean prevB = false;
		String lastAction = "none";

		try {
			while (opModeIsActive()) {
				if (drivetrain != null) {
					drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
				}
				double gyroDeg = Double.NaN;
				if (pinpoint != null) {
					pinpoint.update();
					gyroDeg = pinpoint.getHeading(AngleUnit.DEGREES);
				}

				LLResult result = limelight.getLatestResult();
				if (result != null && result.isValid()
						&& result.getPipelineIndex() == BallVisionConfig.PIPELINE_APRILTAG
						&& result.getStaleness() <= BallVisionConfig.MAX_RESULT_STALENESS_MS) {
					double ts = result.getTimestamp() != 0.0 ? result.getTimestamp() : result.getControlHubTimeStamp();
					if (Double.isNaN(lastFrameTs) || ts != lastFrameTs) {
						lastFrameTs = ts;
						List<TagSighting> sightings = sightings(result);

						tagsOnly = localizer.estimate(sightings, Double.NaN);
						if (tagsOnly.valid && tagsOnly.rmsResidualIn <= ALIGN_MAX_RMS_IN && !Double.isNaN(gyroDeg)) {
							alignSamples.addLast(wrap(tagsOnly.headingDeg - gyroDeg));
							while (alignSamples.size() > ALIGN_WINDOW) {
								alignSamples.removeFirst();
							}
						}
						withGyro = Double.isNaN(headingOffsetDeg) || Double.isNaN(gyroDeg) ? null
								: localizer.estimate(sightings, wrap(gyroDeg + headingOffsetDeg));

						FieldPoseEstimate best = withGyro != null && withGyro.valid ? withGyro : tagsOnly;
						if (best.valid) {
							jitter.addLast(new double[] {best.xIn, best.yIn, best.headingDeg});
							while (jitter.size() > JITTER_WINDOW) {
								jitter.removeFirst();
							}
						}
					}
				} else {
					tagsOnly = null;
					withGyro = null;
				}

				if (gamepad1.a && !prevA) {
					if (pinpoint == null) {
						lastAction = "A: no Pinpoint configured";
					} else if (alignSamples.size() < ALIGN_MIN_SAMPLES) {
						lastAction = "A: need a steady TAGS ONLY fix first (" + alignSamples.size()
								+ "/" + ALIGN_MIN_SAMPLES + " samples)";
					} else {
						headingOffsetDeg = circularMeanDeg(alignSamples);
						jitter.clear();
						lastAction = String.format(Locale.US, "A: gyro aligned (offset %.1f deg)", headingOffsetDeg);
					}
				}
				if (gamepad1.b && !prevB) {
					headingOffsetDeg = Double.NaN;
					jitter.clear();
					lastAction = "B: alignment cleared";
				}
				prevA = gamepad1.a;
				prevB = gamepad1.b;

				boolean aligned = !Double.isNaN(headingOffsetDeg);
				telemetry.addLine("HIVE LOCALIZATION  (A = align gyro, B = clear)");
				telemetry.addData("Mode", aligned ? "HEADING FROM PINPOINT" : "TAGS ONLY");
				FieldPoseEstimate shown = aligned ? withGyro : tagsOnly;
				if (shown == null) {
					telemetry.addData("Pose", "no fresh Limelight frame");
				} else {
					telemetry.addData("Pose", shown.summary());
					if (shown.valid) {
						telemetry.addData("Fit", "tags %d (rejected %d)  fit rms %.1f in  height err %.1f in",
								shown.tagsUsed, shown.tagsRejected, shown.rmsResidualIn, shown.rmsHeightErrorIn);
					}
					telemetry.addData("Clusters", shown.clusters.isEmpty() ? "none" : shown.clusters.toString());
				}
				if (aligned && tagsOnly != null) {
					telemetry.addData("Tags-only check", tagsOnly.summary());
				}
				telemetry.addData("Jitter (last " + jitter.size() + ")", jitterText(jitter));
				if (pinpoint != null) {
					telemetry.addData("Gyro", aligned
							? String.format(Locale.US, "%.1f deg  -> field %.1f deg", gyroDeg, wrap(gyroDeg + headingOffsetDeg))
							: String.format(Locale.US, "%.1f deg (not aligned, %d samples)", gyroDeg, alignSamples.size()));
				}
				telemetry.addData("Last action", lastAction);
				telemetry.addData("Mount", "fwd %.1f left %.1f h %.1f yaw %.1f pitch %.1f",
						Constants.LIMELIGHT_FORWARD_IN, Constants.LIMELIGHT_LEFT_IN, Constants.LIMELIGHT_HEIGHT_IN,
						Constants.LIMELIGHT_YAW_DEG, Constants.LIMELIGHT_PITCH_DEG);
				telemetry.update();
			}
		} finally {
			if (drivetrain != null) {
				drivetrain.driveRaw(0.0, 0.0, 0.0);
			}
			limelight.stop();
		}
	}

	/** HIVE tags of one frame as sightings: 3D range plus crosshair-free angles. */
	private static List<TagSighting> sightings(LLResult result) {
		List<TagSighting> out = new ArrayList<>();
		List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
		if (fiducials == null) {
			return out;
		}
		for (LLResultTypes.FiducialResult f : fiducials) {
			if (f == null || !HiveTagMap.isHiveTag(f.getFiducialId())) {
				continue;
			}
			Pose3D pose = f.getTargetPoseCameraSpace();
			if (pose == null) {
				continue;
			}
			Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
			double range = HiveGeometry.rangeInches(p.x, p.y, p.z);
			if (Double.isNaN(range)) {
				continue;
			}
			out.add(new TagSighting(f.getFiducialId(), range,
					f.getTargetXDegreesNoCrosshair(), f.getTargetYDegreesNoCrosshair()));
		}
		return out;
	}

	private static String jitterText(Deque<double[]> samples) {
		if (samples.size() < 2) {
			return "--";
		}
		double[] mean = new double[3];
		for (double[] s : samples) {
			for (int i = 0; i < 3; i++) {
				mean[i] += s[i] / samples.size();
			}
		}
		double[] var = new double[3];
		for (double[] s : samples) {
			for (int i = 0; i < 3; i++) {
				double d = i == 2 ? wrap(s[i] - mean[i]) : s[i] - mean[i];
				var[i] += d * d / samples.size();
			}
		}
		return String.format(Locale.US, "x +/-%.2f  y +/-%.2f in  h +/-%.1f deg",
				Math.sqrt(var[0]), Math.sqrt(var[1]), Math.sqrt(var[2]));
	}

	private static double circularMeanDeg(Deque<Double> degs) {
		double s = 0.0;
		double c = 0.0;
		for (double d : degs) {
			s += Math.sin(Math.toRadians(d));
			c += Math.cos(Math.toRadians(d));
		}
		return Math.toDegrees(Math.atan2(s, c));
	}

	private static double wrap(double deg) {
		double d = deg % 360.0;
		if (d > 180.0) {
			d -= 360.0;
		} else if (d <= -180.0) {
			d += 360.0;
		}
		return d;
	}

	/** Starts the Limelight on the AprilTag pipeline and repairs it (incl. tag size) if needed. */
	private void startLimelight(Limelight3A limelight) {
		limelight.setPollRateHz(BallVisionConfig.POLL_RATE_HZ);
		limelight.pipelineSwitch(BallVisionConfig.PIPELINE_APRILTAG);
		limelight.start();
		long deadline = System.currentTimeMillis() + 3000;
		while (opModeIsActive() && !limelight.isConnected() && System.currentTimeMillis() < deadline) {
			sleep(50);
		}
		boolean ok = LimelightPipelines.ensureFiducial(limelight, BallVisionConfig.PIPELINE_APRILTAG,
				LimelightPipelines.FTC_FIDUCIAL_FAMILY, Constants.APRILTAG_SIZE_MM);
		if (ok) {
			limelight.pipelineSwitch(BallVisionConfig.PIPELINE_APRILTAG);
		} else {
			telemetry.addLine("PIPELINE NOT FIDUCIAL - run Limelight Pipeline Setup");
			telemetry.update();
			sleep(1500);
		}
	}
}
