package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * Limelight lifecycle + DECODE ball vision. Preserves pipeline 0 for AprilTags
 * (see {@code LimelightPipelines}); pipeline 1 is the neural ball detector.
 * Non-blocking: call {@link #update()} each loop. Never throws when no target is visible.
 */
public class LimelightVision {

	private final Limelight3A limelight;
	private final TargetPersistence persistence = new TargetPersistence();

	private int requestedPipeline = BallVisionConfig.PIPELINE_APRILTAG;
	private boolean started;
	private boolean pipelineReady;
	private double lastAcceptedFrameTs = Double.NaN;

	private List<BallDetection> rawDetections = Collections.emptyList();
	private List<BallDetection> filteredDetections = Collections.emptyList();
	private List<BallGroup> groups = Collections.emptyList();
	private final BallGrouping.RejectCounts rejects = new BallGrouping.RejectCounts();
	private BallTarget target = BallTarget.none();
	private long lastStalenessMs;
	private double lastLatencyMs;
	private int reportedPipeline = -1;
	private boolean debugTelemetry;

	public LimelightVision(HardwareMap hardwareMap) {
		this(hardwareMap.get(Limelight3A.class, "limelight"));
	}

	public LimelightVision(Limelight3A limelight) {
		this.limelight = limelight;
	}

	/** Idempotent start: poll rate + start once. */
	public void start() {
		if (started) {
			return;
		}
		limelight.setPollRateHz(BallVisionConfig.POLL_RATE_HZ);
		limelight.pipelineSwitch(requestedPipeline);
		limelight.start();
		pipelineReady = false;
		started = true;
	}

	public void stop() {
		if (!started) {
			return;
		}
		limelight.stop();
		started = false;
		pipelineReady = false;
		persistence.clear();
		clearFrameState();
	}

	public void useAprilTagPipeline() {
		switchPipeline(BallVisionConfig.PIPELINE_APRILTAG);
	}

	public void useBallDetectionPipeline() {
		switchPipeline(BallVisionConfig.PIPELINE_BALL);
	}

	/** Non-blocking pipeline request; skips no-op switches. */
	public void switchPipeline(int pipeline) {
		if (pipeline == requestedPipeline) {
			return;
		}
		requestedPipeline = pipeline;
		pipelineReady = false;
		lastAcceptedFrameTs = Double.NaN;
		persistence.clear();
		clearFrameState();
		if (started) {
			limelight.pipelineSwitch(pipeline);
		}
	}

	public int getRequestedPipeline() {
		return requestedPipeline;
	}

	public int getReportedPipeline() {
		return reportedPipeline;
	}

	public boolean isPipelineReady() {
		return pipelineReady;
	}

	public void setDesiredBallColor(BallColor color) {
		persistence.setDesiredColor(color);
	}

	public void setDebugTelemetry(boolean enabled) {
		debugTelemetry = enabled;
	}

	/**
	 * Call once per control loop. Reads Limelight, rejects stale/wrong-pipeline frames,
	 * groups balls when on pipeline 1, updates sticky target.
	 */
	public void update() {
		long now = System.currentTimeMillis();
		LLResult result = limelight.getLatestResult();
		LLStatus status = limelight.getStatus();
		if (status != null) {
			reportedPipeline = status.getPipelineIndex();
		}

		if (result == null || !result.isValid()) {
			loseTarget(now);
			return;
		}

		reportedPipeline = result.getPipelineIndex();
		lastStalenessMs = result.getStaleness();
		lastLatencyMs = result.getCaptureLatency() + result.getTargetingLatency();

		if (result.getPipelineIndex() != requestedPipeline) {
			pipelineReady = false;
			loseTarget(now);
			return;
		}

		if (lastStalenessMs > BallVisionConfig.MAX_RESULT_STALENESS_MS) {
			loseTarget(now);
			return;
		}

		double frameTs = result.getTimestamp();
		if (frameTs == 0.0) {
			// Some firmware omits the "ts" field, which would pin every frame at 0.0 and make
			// newFrame permanently false after the first one, silently freezing the pipeline.
			frameTs = result.getControlHubTimeStamp();
		}
		boolean newFrame = Double.isNaN(lastAcceptedFrameTs) || frameTs != lastAcceptedFrameTs;
		if (!pipelineReady) {
			pipelineReady = true;
		}

		if (requestedPipeline != BallVisionConfig.PIPELINE_BALL) {
			clearFrameState();
			target = BallTarget.none();
			if (newFrame) {
				lastAcceptedFrameTs = frameTs;
			}
			return;
		}

		if (!newFrame) {
			target = persistence.update(now, Double.NaN, Optional.empty());
			return;
		}

		lastAcceptedFrameTs = frameTs;
		rawDetections = parseDetections(result);
		filteredDetections = BallGrouping.filterDetections(rawDetections, rejects);
		groups = BallGrouping.groupDetections(filteredDetections, persistence.getDesiredColor());
		target = persistence.update(now, frameTs, groups);
	}

	/** No usable frame this loop: drop frame data but let persistence hold a recent target. */
	private void loseTarget(long now) {
		clearFrameState();
		target = persistence.update(now, Double.NaN, Optional.empty());
	}

	public List<BallDetection> getBallDetections() {
		return filteredDetections;
	}

	public List<BallGroup> getBallGroups() {
		return groups;
	}

	public Optional<BallGroup> getBestBallGroup() {
		return BallGrouping.selectBestGroup(groups);
	}

	public Optional<BallGroup> getBestBallGroup(BallColor desiredColor) {
		return BallGrouping.selectBestGroup(groups, desiredColor);
	}

	public BallTarget getTarget() {
		return target;
	}

	public Optional<BallGroup> getPersistedGroup() {
		return persistence.getStickyGroup();
	}

	public Limelight3A getLimelight() {
		return limelight;
	}

	public void addTelemetry(Telemetry telemetry) {
		telemetry.addData("LL pipe req/rep", "%d / %d", requestedPipeline, reportedPipeline);
		telemetry.addData("LL ready", pipelineReady);
		telemetry.addData("LL stale/lat ms", "%d / %.0f", lastStalenessMs, lastLatencyMs);
		if (requestedPipeline == BallVisionConfig.PIPELINE_BALL) {
			telemetry.addData("Balls raw/filt/groups", "%d / %d / %d",
					rawDetections.size(), filteredDetections.size(), groups.size());
			if (rejects.total() > 0) {
				telemetry.addData("Rejected", rejects.toString());
			}
			BallTarget t = target;
			if (t.isValid()) {
				telemetry.addData("Target", "sz=%d tx=%.1f ty=%.1f age=%dms %s",
						t.getGroupSize(), t.getHorizontalErrorDeg(), t.getVerticalAngleDeg(),
						t.getAgeMs(), t.isFresh() ? "fresh" : "held");
				Optional<BallGroup> sticky = persistence.getStickyGroup();
				double maxTa = sticky.isPresent()
						? BallGrouping.apparentCloseness(sticky.get())
						: 0.0;
				telemetry.addData("Target G/P maxTa score", "%d/%d ta=%.3f %.2f",
						t.getGreenCount(), t.getPurpleCount(), maxTa, t.getScore());
			} else {
				telemetry.addData("Target", "none");
			}
		}
		if (debugTelemetry) {
			// Walk the raw list, not the filtered one, so a rejected detection still shows the
			// values that got it dropped. This is the view that diagnoses an empty filter.
			for (int i = 0; i < rawDetections.size(); i++) {
				BallDetection d = rawDetections.get(i);
				telemetry.addData("Det" + i, "%s conf=%.2f ta=%.3f tx=%.1f ty=%.1f box=%s %s",
						d.getClassName(), d.getConfidence(), d.getTargetArea(),
						d.getTxDeg(), d.getTyDeg(), d.hasPixelBox() ? "y" : "n",
						rejectReason(d));
			}
		}
	}

	/** Which filter gate a raw detection fails, for debug telemetry. */
	private static String rejectReason(BallDetection d) {
		if (d.getConfidence() < BallVisionConfig.MIN_CONFIDENCE) {
			return "DROP:conf";
		}
		if (d.getTargetArea() < BallVisionConfig.MIN_TARGET_AREA) {
			return "DROP:area";
		}
		if (d.getColor() != BallColor.GREEN && d.getColor() != BallColor.PURPLE) {
			return "DROP:color";
		}
		if (BallGrouping.isNearImageEdge(d)) {
			return "DROP:edge";
		}
		return "ok";
	}

	private void clearFrameState() {
		rawDetections = Collections.emptyList();
		filteredDetections = Collections.emptyList();
		groups = Collections.emptyList();
	}

	private static List<BallDetection> parseDetections(LLResult result) {
		List<LLResultTypes.DetectorResult> dets = result.getDetectorResults();
		if (dets == null || dets.isEmpty()) {
			return Collections.emptyList();
		}
		List<BallDetection> out = new ArrayList<>(dets.size());
		double frameTs = result.getTimestamp();
		long hubTs = result.getControlHubTimeStamp();
		for (LLResultTypes.DetectorResult dr : dets) {
			if (dr == null) {
				continue;
			}
			List<List<Double>> corners = dr.getTargetCorners();
			double cx;
			double cy;
			double w;
			double h;
			boolean hasBox;
			if (corners != null && corners.size() >= 4) {
				double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
				double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
				for (List<Double> pt : corners) {
					if (pt == null || pt.size() < 2) {
						continue;
					}
					minX = Math.min(minX, pt.get(0));
					maxX = Math.max(maxX, pt.get(0));
					minY = Math.min(minY, pt.get(1));
					maxY = Math.max(maxY, pt.get(1));
				}
				if (Double.isFinite(minX) && Double.isFinite(maxX)) {
					cx = (minX + maxX) / 2.0;
					cy = (minY + maxY) / 2.0;
					w = Math.max(1.0, maxX - minX);
					h = Math.max(1.0, maxY - minY);
					hasBox = true;
				} else {
					cx = dr.getTargetXPixels();
					cy = dr.getTargetYPixels();
					w = 0.0;
					h = 0.0;
					hasBox = false;
				}
			} else {
				// No corner data, so box size is unknown; grouping falls back to tx/ty angles.
				cx = dr.getTargetXPixels();
				cy = dr.getTargetYPixels();
				w = 0.0;
				h = 0.0;
				hasBox = false;
			}
			out.add(new BallDetection(
					dr.getClassName(),
					dr.getConfidence(),
					dr.getTargetXDegrees(),
					dr.getTargetYDegrees(),
					dr.getTargetArea(),
					corners,
					frameTs,
					hubTs,
					cx, cy, w, h,
					hasBox));
		}
		return out;
	}
}
