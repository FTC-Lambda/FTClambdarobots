package org.firstinspires.ftc.teamcode.vision;

import java.util.Collections;
import java.util.List;
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
		return update(nowHubMs, frameTimestamp,
				candidate != null && candidate.isPresent()
						? Collections.singletonList(candidate.get())
						: Collections.<BallGroup>emptyList());
	}

	/**
	 * @param nowHubMs current Control Hub time (ms)
	 * @param frameTimestamp Limelight local frame timestamp; NaN if no new frame
	 * @param freshGroups all scored groups from a fresh frame; empty when nothing is visible
	 */
	public BallTarget update(long nowHubMs, double frameTimestamp, List<BallGroup> freshGroups) {
		boolean freshFrame = !Double.isNaN(frameTimestamp)
				&& (Double.isNaN(lastFrameTimestamp) || frameTimestamp != lastFrameTimestamp);

		if (freshFrame && freshGroups != null && !freshGroups.isEmpty()) {
			stickyGroup = chooseGroup(freshGroups);
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

	/**
	 * Picks the closest group by apparent size ({@code ta}). Stays on the tracked cluster
	 * unless another is clearly closer, so two similarly sized piles do not flip the aim.
	 * Always returns a group from the current frame so reported angles are never stale.
	 */
	private BallGroup chooseGroup(List<BallGroup> freshGroups) {
		Optional<BallGroup> closestOpt = BallGrouping.selectClosestGroup(freshGroups, desiredColor);
		if (!closestOpt.isPresent()) {
			return null;
		}
		BallGroup closest = closestOpt.get();

		BallGroup bestAssociated = null;
		for (BallGroup g : freshGroups) {
			if (g == null || stickyGroup == null || !areAssociated(stickyGroup, g)) {
				continue;
			}
			if (desiredColor == BallColor.GREEN || desiredColor == BallColor.PURPLE) {
				int count = desiredColor == BallColor.GREEN ? g.getGreenCount() : g.getPurpleCount();
				if (count <= 0) {
					continue;
				}
			}
			if (BallGrouping.isCloser(g, bestAssociated)) {
				bestAssociated = g;
			}
		}
		if (bestAssociated == null) {
			return closest;
		}

		double closestArea = BallGrouping.apparentCloseness(closest);
		double stickyArea = BallGrouping.apparentCloseness(bestAssociated);
		if (stickyArea <= 0.0) {
			return closest;
		}
		return closestArea > stickyArea * BallVisionConfig.CLOSEST_AREA_SWITCH_RATIO
				? closest
				: bestAssociated;
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
