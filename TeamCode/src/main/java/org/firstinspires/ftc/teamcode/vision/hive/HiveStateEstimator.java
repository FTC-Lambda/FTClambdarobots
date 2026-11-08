package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.Arrays;
import java.util.List;

/**
 * Turns per-frame AprilTag heights of our alliance's HIVE into a debounced HIVE state.
 *
 * <p>Per frame, each CELL's median tag height is classified RAISED / LOWERED / AMBIGUOUS against
 * the configured threshold with hysteresis. A raised FAR CELL or a lowered AUDIENCE CELL means
 * FAR_CELL_UP; a raised AUDIENCE CELL or a lowered FAR CELL means AUDIENCE_CELL_UP. A new state
 * is only confirmed after {@code confirmFrames} consecutive voting frames agree. The confirmed
 * state is held indefinitely (the HIVE physically stays put); use {@link #isFresh(long)} to see
 * whether it was recently supported by a camera frame.
 *
 * <p>Pure logic: no FTC SDK dependencies. Call {@link #update} once per NEW camera frame.
 */
public final class HiveStateEstimator {

	private Alliance alliance;
	private HiveDetectionConfig config;

	private HiveState confirmed = HiveState.UNKNOWN;
	private long lastSupportMs;
	private HiveState pending = HiveState.UNKNOWN;
	private int pendingFrames;
	private HiveState lastFrameVote = HiveState.UNKNOWN;

	private final double[] cellHeightIn = {Double.NaN, Double.NaN};
	private final CellPosition[] cellPosition = {CellPosition.NOT_SEEN, CellPosition.NOT_SEEN};
	private final int[] cellTagCount = {0, 0};

	public HiveStateEstimator(Alliance alliance, HiveDetectionConfig config) {
		this.alliance = requireAlliance(alliance);
		this.config = requireConfig(config);
	}

	public void setAlliance(Alliance a) {
		this.alliance = requireAlliance(a);
		reset();
	}

	public Alliance getAlliance() {
		return alliance;
	}

	/** Swaps tuning; keeps the confirmed state but discards any pending confirmation. */
	public void setConfig(HiveDetectionConfig c) {
		this.config = requireConfig(c);
		clearPending();
	}

	public HiveDetectionConfig getConfig() {
		return config;
	}

	public void reset() {
		confirmed = HiveState.UNKNOWN;
		lastSupportMs = 0L;
		lastFrameVote = HiveState.UNKNOWN;
		clearPending();
		for (int i = 0; i < 2; i++) {
			cellHeightIn[i] = Double.NaN;
			cellPosition[i] = CellPosition.NOT_SEEN;
			cellTagCount[i] = 0;
		}
	}

	/**
	 * @param nowMs current time (ms)
	 * @param observations tags from one new camera frame; may be null or empty
	 * @return the confirmed state after processing the frame
	 */
	public HiveState update(long nowMs, List<HiveTagObservation> observations) {
		HiveDetectionConfig cfg = config;
		int size = observations == null ? 0 : observations.size();
		double[][] heights = new double[2][size];
		int[] counts = new int[2];
		if (observations != null) {
			for (HiveTagObservation o : observations) {
				if (o == null || o.getAlliance() != alliance || o.getCell() == null) {
					continue;
				}
				double h = o.getHeightIn();
				if (Double.isNaN(h) || Double.isInfinite(h)
						|| h < cfg.minPlausibleHeightIn || h > cfg.maxPlausibleHeightIn) {
					continue;
				}
				int idx = o.getCell().ordinal();
				heights[idx][counts[idx]++] = h;
			}
		}

		HiveState[] votes = new HiveState[2];
		for (HiveCell cell : HiveCell.values()) {
			int idx = cell.ordinal();
			cellTagCount[idx] = counts[idx];
			if (counts[idx] >= cfg.minTagsPerCell) {
				double median = median(heights[idx], counts[idx]);
				cellHeightIn[idx] = median;
				cellPosition[idx] = classify(median, cfg);
			} else {
				cellHeightIn[idx] = Double.NaN;
				cellPosition[idx] = CellPosition.NOT_SEEN;
			}
			votes[idx] = voteOf(cell, cellPosition[idx]);
		}

		HiveState vote = combine(votes[HiveCell.FAR.ordinal()],
				votes[HiveCell.AUDIENCE.ordinal()]);
		lastFrameVote = vote;
		if (vote != HiveState.UNKNOWN) {
			applyVote(nowMs, vote, cfg);
		}
		return confirmed;
	}

	private void applyVote(long nowMs, HiveState vote, HiveDetectionConfig cfg) {
		if (vote == confirmed) {
			lastSupportMs = nowMs;
			clearPending();
			return;
		}
		if (vote == pending) {
			pendingFrames++;
		} else {
			pending = vote;
			pendingFrames = 1;
		}
		if (pendingFrames >= cfg.confirmFrames) {
			confirmed = vote;
			lastSupportMs = nowMs;
			clearPending();
		}
	}

	private void clearPending() {
		pending = HiveState.UNKNOWN;
		pendingFrames = 0;
	}

	private static CellPosition classify(double medianIn, HiveDetectionConfig cfg) {
		if (medianIn >= cfg.raisedThresholdIn + cfg.hysteresisIn) {
			return CellPosition.RAISED;
		}
		if (medianIn <= cfg.raisedThresholdIn - cfg.hysteresisIn) {
			return CellPosition.LOWERED;
		}
		return CellPosition.AMBIGUOUS;
	}

	private static HiveState voteOf(HiveCell cell, CellPosition position) {
		if (position == CellPosition.RAISED) {
			return HiveState.withUp(cell);
		}
		if (position == CellPosition.LOWERED) {
			return HiveState.withUp(cell.other());
		}
		return HiveState.UNKNOWN;
	}

	/** UNKNOWN means no vote; disagreeing cells cancel out. */
	private static HiveState combine(HiveState a, HiveState b) {
		if (a == HiveState.UNKNOWN) {
			return b;
		}
		if (b == HiveState.UNKNOWN || a == b) {
			return a;
		}
		return HiveState.UNKNOWN;
	}

	private static double median(double[] values, int count) {
		double[] sorted = Arrays.copyOf(values, count);
		Arrays.sort(sorted);
		if (count % 2 == 1) {
			return sorted[count / 2];
		}
		return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0;
	}

	/** Last confirmed state; UNKNOWN until first confirmation. Not cleared by staleness. */
	public HiveState getState() {
		return confirmed;
	}

	public boolean isFresh(long nowMs) {
		return confirmed != HiveState.UNKNOWN && nowMs - lastSupportMs <= config.staleAfterMs;
	}

	/** Milliseconds since the confirmed state was last supported, or -1 if never confirmed. */
	public long getAgeMs(long nowMs) {
		return confirmed == HiveState.UNKNOWN ? -1L : nowMs - lastSupportMs;
	}

	/** Median plausible tag height of the CELL in the last update; NaN if not seen. */
	public double getCellHeightIn(HiveCell c) {
		return cellHeightIn[c.ordinal()];
	}

	public CellPosition getCellPosition(HiveCell c) {
		return cellPosition[c.ordinal()];
	}

	/** Number of plausible tags of the CELL used in the last update. */
	public int getCellTagCount(HiveCell c) {
		return cellTagCount[c.ordinal()];
	}

	/**
	 * The single-frame vote from the last update, before confirmation: what this one frame
	 * says (UNKNOWN when no CELL classified or the CELLs contradicted each other).
	 */
	public HiveState getLastFrameVote() {
		return lastFrameVote;
	}

	public HiveState getPendingState() {
		return pending;
	}

	public int getPendingFrames() {
		return pendingFrames;
	}

	private static Alliance requireAlliance(Alliance a) {
		if (a == null) {
			throw new IllegalArgumentException("alliance must not be null");
		}
		return a;
	}

	private static HiveDetectionConfig requireConfig(HiveDetectionConfig c) {
		if (c == null) {
			throw new IllegalArgumentException("config must not be null");
		}
		return c;
	}
}
