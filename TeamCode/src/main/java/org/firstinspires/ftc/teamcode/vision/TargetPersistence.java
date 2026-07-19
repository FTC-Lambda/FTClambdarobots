package org.firstinspires.ftc.teamcode.vision;

import java.util.Optional;

/**
 * Lightweight sticky targeting across low-FPS neural frames.
 * Associates new groups by center proximity; clears after vision-loss timeout.
 */
public final class TargetPersistence {

	private BallGroup stickyGroup;
	private long lastSeenHubMs;
	private double lastFrameTimestamp = Double.NaN;
	private BallColor desiredColor;

	public void clear() {
		stickyGroup = null;
		lastSeenHubMs = 0L;
		lastFrameTimestamp = Double.NaN;
	}

	public void setDesiredColor(BallColor desiredColor) {
		this.desiredColor = desiredColor;
	}

	public BallColor getDesiredColor() {
		return desiredColor;
	}

	/**
	 * @param nowHubMs current Control Hub time (ms)
	 * @param frameTimestamp Limelight local frame timestamp; NaN if no new frame
	 * @param candidate optional newly computed best group from a fresh frame
	 */
	public BallTarget update(long nowHubMs, double frameTimestamp, Optional<BallGroup> candidate) {
		boolean freshFrame = !Double.isNaN(frameTimestamp)
				&& (Double.isNaN(lastFrameTimestamp) || frameTimestamp != lastFrameTimestamp);

		if (candidate != null && candidate.isPresent() && freshFrame) {
			BallGroup next = candidate.get();
			stickyGroup = next;
			lastSeenHubMs = nowHubMs;
			lastFrameTimestamp = frameTimestamp;
			return toTarget(stickyGroup, true, 0L);
		}

		if (stickyGroup == null) {
			return BallTarget.none();
		}

		long age = Math.max(0L, nowHubMs - lastSeenHubMs);
		if (age > BallVisionConfig.VISION_LOSS_TIMEOUT_MS) {
			clear();
			return BallTarget.none();
		}
		return toTarget(stickyGroup, false, age);
	}

	public static boolean areAssociated(BallGroup previous, BallGroup next) {
		if (previous == null || next == null) {
			return false;
		}
		double dTx = previous.getWeightedTxDeg() - next.getWeightedTxDeg();
		double dTy = previous.getWeightedTyDeg() - next.getWeightedTyDeg();
		return Math.hypot(dTx, dTy) <= BallVisionConfig.PERSIST_ASSOCIATION_DEG;
	}

	public Optional<BallGroup> getStickyGroup() {
		return Optional.ofNullable(stickyGroup);
	}

	private BallTarget toTarget(BallGroup g, boolean fresh, long ageMs) {
		return new BallTarget(
				true,
				fresh,
				g.getWeightedTxDeg(),
				g.getWeightedTyDeg(),
				g.getAverageConfidence(),
				g.getSize(),
				ageMs,
				g.getScore(),
				g.getGreenCount(),
				g.getPurpleCount(),
				desiredColor);
	}
}
