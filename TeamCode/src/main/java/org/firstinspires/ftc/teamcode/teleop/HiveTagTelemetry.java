package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
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
import org.firstinspires.ftc.teamcode.vision.hive.HiveGeometry;
import org.firstinspires.ftc.teamcode.vision.hive.HiveTagMap;
import org.firstinspires.ftc.teamcode.vision.hive.HiveTagReadout;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * Raw Limelight telemetry for the HIVE AprilTags only (IDs 30-45). Every other tag is ignored
 * and only counted. There is no state logic: this shows exactly what the camera reports, for
 * checking range, viewing angles and tag heights around the real HIVE.
 *
 * <p>Rows: "33 R-F  tx ty ta r h". tx/ty are degrees from the lens center (crosshair offset
 * removed), ta is tag area in % of the image, r is 3D range and h is height above the tiles
 * (both inches; h uses Constants.LIMELIGHT_HEIGHT_IN / LIMELIGHT_PITCH_DEG). "--" means the
 * Limelight's 3D solve failed for that tag.</p>
 *
 * <p>Controls: Y = toggle detail rows (camera-space x/y/z and tag yaw). Sticks drive if the
 * drivetrain is configured; it runs with only the Limelight connected. Nothing moves in INIT.</p>
 */
@TeleOp(name = "Hive Tag Telemetry", group = "Vision")
public class HiveTagTelemetry extends LinearOpMode {

	@Override
	public void runOpMode() {
		Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
		Drivetrain drivetrain = Drivetrain.tryCreate(hardwareMap);

		telemetry.addLine("Hive Tag Telemetry - shows HIVE tags 30-45 only. Press START.");
		telemetry.addData("Drivetrain", drivetrain != null ? "found" : "not configured - vision only");
		telemetry.update();
		waitForStart();
		if (isStopRequested()) {
			return;
		}

		startLimelight(limelight);

		boolean detail = false;
		boolean prevY = false;
		double lastFrameTs = Double.NaN;
		int framesThisWindow = 0;
		long windowStartMs = System.currentTimeMillis();
		double measuredFps = 0.0;

		try {
			while (opModeIsActive()) {
				if (gamepad1.y && !prevY) {
					detail = !detail;
				}
				prevY = gamepad1.y;
				if (drivetrain != null) {
					drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
				}

				LLStatus status = limelight.getStatus();
				LLResult result = limelight.getLatestResult();

				telemetry.addLine("HIVE TAGS ONLY (30-45)   Y = detail " + (detail ? "on" : "off"));
				if (status != null) {
					telemetry.addData("Limelight", "%s  fps %.0f  temp %.0fC  cpu %.0f%%",
							limelight.isConnected() ? "connected" : "NOT CONNECTED",
							status.getFps(), status.getTemp(), status.getCpu());
				}

				if (result == null) {
					telemetry.addLine("No result from Limelight");
					telemetry.update();
					continue;
				}

				// Count new frames to show the rate results actually arrive at the Control Hub.
				double ts = result.getTimestamp() != 0.0 ? result.getTimestamp() : result.getControlHubTimeStamp();
				if (Double.isNaN(lastFrameTs) || ts != lastFrameTs) {
					lastFrameTs = ts;
					framesThisWindow++;
				}
				long now = System.currentTimeMillis();
				if (now - windowStartMs >= 1000) {
					measuredFps = framesThisWindow * 1000.0 / (now - windowStartMs);
					framesThisWindow = 0;
					windowStartMs = now;
				}

				telemetry.addData("Pipeline", "%d%s  stale %dms  latency %.0fms  frames/s %.0f",
						result.getPipelineIndex(),
						result.getPipelineIndex() == BallVisionConfig.PIPELINE_APRILTAG ? "" : " (NOT AprilTag!)",
						result.getStaleness(),
						result.getCaptureLatency() + result.getTargetingLatency(),
						measuredFps);

				List<LLResultTypes.FiducialResult> hive = new ArrayList<>();
				List<Integer> ignored = new ArrayList<>();
				int[] perCell = new int[4]; // R-F, R-A, B-A, B-F
				List<LLResultTypes.FiducialResult> all = result.isValid() ? result.getFiducialResults() : null;
				if (all != null) {
					for (LLResultTypes.FiducialResult f : all) {
						int id = f.getFiducialId();
						if (HiveTagMap.isHiveTag(id)) {
							hive.add(f);
							perCell[(id - 30) / 4]++;
						} else {
							ignored.add(id);
						}
					}
				}
				Collections.sort(hive, new Comparator<LLResultTypes.FiducialResult>() {
					@Override
					public int compare(LLResultTypes.FiducialResult a, LLResultTypes.FiducialResult b) {
						return Integer.compare(a.getFiducialId(), b.getFiducialId());
					}
				});

				telemetry.addData("Seen", "R-F %d  R-A %d  B-A %d  B-F %d   ignored %s",
						perCell[0], perCell[1], perCell[2], perCell[3],
						ignored.isEmpty() ? "none" : ignored.toString());
				telemetry.addData("Mount", String.format(Locale.US, "h %.2f in  pitch %.1f deg (Constants)",
						Constants.LIMELIGHT_HEIGHT_IN, Constants.LIMELIGHT_PITCH_DEG));

				if (hive.isEmpty()) {
					telemetry.addLine("No HIVE tags in view");
				}
				for (LLResultTypes.FiducialResult f : hive) {
					double x = Double.NaN;
					double y = Double.NaN;
					double z = Double.NaN;
					double yaw = Double.NaN;
					Pose3D pose = f.getTargetPoseCameraSpace();
					if (pose != null) {
						Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
						x = p.x;
						y = p.y;
						z = p.z;
						yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
					}
					double tx = f.getTargetXDegreesNoCrosshair();
					double ty = f.getTargetYDegreesNoCrosshair();
					double range = HiveGeometry.rangeInches(x, y, z);
					double height = HiveGeometry.tagHeightInches(range, tx, ty,
							Constants.LIMELIGHT_HEIGHT_IN, Constants.LIMELIGHT_PITCH_DEG);
					telemetry.addData(HiveTagReadout.label(f.getFiducialId()),
							HiveTagReadout.row(tx, ty, f.getTargetArea(), range, height));
					if (detail) {
						telemetry.addData("   ", HiveTagReadout.detail(x, y, z, yaw));
					}
				}
				telemetry.update();
			}
		} finally {
			if (drivetrain != null) {
				drivetrain.driveRaw(0.0, 0.0, 0.0);
			}
			limelight.stop();
		}
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
