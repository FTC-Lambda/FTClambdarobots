package org.firstinspires.ftc.teamcode.vision.hive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.vision.BallVisionConfig;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * FTC-SDK glue for HIVE state detection: reads the Limelight's AprilTag pipeline, turns every
 * visible HIVE tag into a height-above-floor observation, and feeds a {@link HiveStateEstimator}.
 *
 * <p>This class only READS. The OpMode owns the Limelight lifecycle (poll rate, pipeline switch,
 * start/stop, {@code LimelightPipelines.ensureFiducial}). Call {@link #update()} once per loop.
 * Height needs the Limelight's 3D solve (fiducial_skip3d = 0) plus a measured camera mount
 * ({@link Constants#LIMELIGHT_HEIGHT_IN}, {@link Constants#LIMELIGHT_PITCH_DEG}).</p>
 */
public class HiveDetector {

	private final Limelight3A limelight;
	private final HiveStateEstimator estimator;
	private Alliance alliance;
	private HiveDetectionConfig config;
	private double cameraHeightIn;
	private double cameraPitchDeg;

	private double lastAcceptedFrameTs = Double.NaN;
	private List<HiveTagObservation> lastObservations = Collections.emptyList();
	private int lastNo3dCount;
	private int framesProcessed;

	public HiveDetector(Limelight3A limelight, Alliance alliance, HiveDetectionConfig config,
						double cameraHeightIn, double cameraPitchDeg) {
		this.limelight = limelight;
		this.alliance = alliance;
		this.config = config == null ? HiveDetectionConfig.defaults() : config;
		this.cameraHeightIn = cameraHeightIn;
		this.cameraPitchDeg = cameraPitchDeg;
		this.estimator = new HiveStateEstimator(alliance, this.config);
	}

	public HiveDetector(Limelight3A limelight, Alliance alliance) {
		this(limelight, alliance, HiveDetectionConfig.defaults(),
				Constants.LIMELIGHT_HEIGHT_IN, Constants.LIMELIGHT_PITCH_DEG);
	}

	/**
	 * Call once per control loop. Ignores missing/invalid, wrong-pipeline, stale and repeated
	 * frames; for each new frame builds observations for all HIVE tags and updates the estimator.
	 */
	public void update() {
		long now = System.currentTimeMillis();
		LLResult result = limelight.getLatestResult();

		if (result == null || !result.isValid()) {
			lastObservations = Collections.emptyList();
			lastNo3dCount = 0;
			return;
		}
		if (result.getPipelineIndex() != BallVisionConfig.PIPELINE_APRILTAG) {
			lastObservations = Collections.emptyList();
			lastNo3dCount = 0;
			return;
		}
		if (result.getStaleness() > BallVisionConfig.MAX_RESULT_STALENESS_MS) {
			lastObservations = Collections.emptyList();
			lastNo3dCount = 0;
			return;
		}

		double frameTs = result.getTimestamp();
		if (frameTs == 0.0) {
			// Some firmware omits "ts"; fall back so frames are not all treated as repeats.
			frameTs = result.getControlHubTimeStamp();
		}
		if (!Double.isNaN(lastAcceptedFrameTs) && frameTs == lastAcceptedFrameTs) {
			return; // same frame as last loop: keep the previous observations for display
		}
		lastAcceptedFrameTs = frameTs;
		framesProcessed++;

		List<HiveTagObservation> observations = new ArrayList<>();
		int no3d = 0;
		List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
		if (fiducials != null) {
			for (LLResultTypes.FiducialResult f : fiducials) {
				if (f == null) {
					continue;
				}
				int id = f.getFiducialId();
				if (!HiveTagMap.isHiveTag(id)) {
					continue;
				}
				if (f.getTargetPoseCameraSpace() == null) {
					no3d++;
					continue;
				}
				Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
				double range = HiveGeometry.rangeInches(p.x, p.y, p.z);
				double height = HiveGeometry.tagHeightInches(range,
						f.getTargetXDegreesNoCrosshair(), f.getTargetYDegreesNoCrosshair(),
						cameraHeightIn, cameraPitchDeg);
				if (Double.isNaN(height)) {
					no3d++;
					continue;
				}
				observations.add(new HiveTagObservation(
						id, HiveTagMap.allianceOf(id), HiveTagMap.cellOf(id), height));
			}
		}
		lastObservations = Collections.unmodifiableList(observations);
		lastNo3dCount = no3d;
		estimator.update(now, observations);
	}

	public HiveState getState() {
		return estimator.getState();
	}

	public boolean isFresh() {
		return estimator.isFresh(System.currentTimeMillis());
	}

	/** Milliseconds since the confirmed state was last supported, or -1 if never confirmed. */
	public long getAgeMs() {
		return estimator.getAgeMs(System.currentTimeMillis());
	}

	public HiveStateEstimator getEstimator() {
		return estimator;
	}

	/** HIVE tags (both alliances) with valid height from the last accepted frame. */
	public List<HiveTagObservation> getLastObservations() {
		return lastObservations;
	}

	/** HIVE tags in the last accepted frame skipped because no 3D pose was available. */
	public int getLastNo3dCount() {
		return lastNo3dCount;
	}

	public int getFramesProcessed() {
		return framesProcessed;
	}

	public void setAlliance(Alliance alliance) {
		this.alliance = alliance;
		estimator.setAlliance(alliance);
		lastObservations = Collections.emptyList();
		lastNo3dCount = 0;
	}

	public Alliance getAlliance() {
		return alliance;
	}

	public void setConfig(HiveDetectionConfig config) {
		this.config = config == null ? HiveDetectionConfig.defaults() : config;
		estimator.setConfig(this.config);
	}

	public HiveDetectionConfig getConfig() {
		return config;
	}

	public void setCameraMount(double heightIn, double pitchDeg) {
		this.cameraHeightIn = heightIn;
		this.cameraPitchDeg = pitchDeg;
	}

	public double getCameraHeightIn() {
		return cameraHeightIn;
	}

	public double getCameraPitchDeg() {
		return cameraPitchDeg;
	}

	public void addTelemetry(Telemetry t) {
		long now = System.currentTimeMillis();
		HiveState state = estimator.getState();
		String who = "YOUR HIVE (" + alliance + "): ";
		if (state == HiveState.UNKNOWN) {
			t.addData("HIVE", who + "UNKNOWN");
		} else {
			String freshness = estimator.isFresh(now)
					? "fresh " + estimator.getAgeMs(now) + "ms"
					: "STALE " + estimator.getAgeMs(now) + "ms";
			t.addData("HIVE", who + HiveLabels.describe(alliance, state) + "  [" + freshness + "]");
		}
		t.addData("This frame says", HiveLabels.stateName(estimator.getLastFrameVote()));
		for (HiveCell cell : HiveCell.values()) {
			double h = estimator.getCellHeightIn(cell);
			t.addData(HiveLabels.shortCode(alliance, cell) + " " + cell + " cell",
					"tags=%d  median=%s in  %s",
					estimator.getCellTagCount(cell),
					Double.isNaN(h) ? "--" : String.format("%.1f", h),
					estimator.getCellPosition(cell));
		}
		HiveState pending = estimator.getPendingState();
		t.addData("Pending", "%s  %d/%d frames", pending, estimator.getPendingFrames(),
				config.confirmFrames);
		t.addData("No-3D tags (this frame)", lastNo3dCount);
		StringBuilder ids = new StringBuilder();
		for (HiveTagObservation o : lastObservations) {
			if (ids.length() > 0) {
				ids.append(", ");
			}
			ids.append(o.getTagId());
		}
		t.addData("Hive tag IDs seen", ids.length() == 0 ? "none" : ids.toString());
		t.addData("Frames processed", framesProcessed);
		t.addData("Camera mount", "h=%.2f in  pitch=%.1f deg", cameraHeightIn, cameraPitchDeg);
	}
}
