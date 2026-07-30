package org.firstinspires.ftc.teamcode.vision;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

/**
 * Lightweight sticky targeting across low-FPS neural frames.
 * Associates new groups by center proximity; clears after vision-loss timeout.
 */
public final class TargetPersistence {

	private BallTargetingConfig config;
	private BallGroup stickyGroup;
	private BallGroup pendingGroup;
	private int pendingConfirmationFrames;
	private double filteredTxDeg;
	private double filteredTyDeg;
	private double rawLockedTxDeg;
	private double rawLockedTyDeg;
	private boolean hasFilteredAim;
	private long lastSeenHubMs;
	private double lastFrameTimestamp = Double.NaN;
	private BallColor desiredColor;

	public TargetPersistence() {
		this(BallTargetingConfig.defaults());
	}

	public TargetPersistence(BallTargetingConfig config) {
		this.config = requireConfig(config);
	}

	public void clear() {
		stickyGroup = null;
		clearPending();
		filteredTxDeg = 0.0;
		filteredTyDeg = 0.0;
		rawLockedTxDeg = 0.0;
		rawLockedTyDeg = 0.0;
		hasFilteredAim = false;
		lastSeenHubMs = 0L;
		lastFrameTimestamp = Double.NaN;
	}

	public void setDesiredColor(BallColor desiredColor) {
		if (this.desiredColor != desiredColor) {
			clear();
		}
		this.desiredColor = desiredColor;
	}

	public BallColor getDesiredColor() {
		return desiredColor;
	}

	public void setConfig(BallTargetingConfig config) {
		this.config = requireConfig(config);
		clear();
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

		if (freshFrame) {
			lastFrameTimestamp = frameTimestamp;
			BallTarget freshTarget = processFreshFrame(nowHubMs, freshGroups);
			if (freshTarget != null) {
				return freshTarget;
			}
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
	 * Keeps identity continuity with the in-gate observation nearest the previous lock.
	 * Other groups may challenge that lock, but promotion requires consecutive fresh frames.
	 */
	private BallTarget processFreshFrame(long nowHubMs, List<BallGroup> freshGroups) {
		boolean enforceDesiredColor = hasDesiredColorGroup(freshGroups);
		if (stickyGroup == null) {
			BallGroup acquired = selectBestGroup(freshGroups, null, enforceDesiredColor);
			if (acquired == null) {
				clearPending();
				return null;
			}
			stickyGroup = acquired;
			lastSeenHubMs = nowHubMs;
			updateFilteredAim(acquired, true);
			clearPending();
			return toTarget(stickyGroup, true, 0L);
		}

		BallGroup currentObservation =
				selectCurrentObservation(freshGroups, enforceDesiredColor);
		BallGroup challenger =
				selectBestGroup(freshGroups, currentObservation, enforceDesiredColor);

		if (currentObservation != null) {
			stickyGroup = currentObservation;
			lastSeenHubMs = nowHubMs;
			updateFilteredAim(currentObservation, false);
		}

		BallGroup incumbent = currentObservation != null ? currentObservation : stickyGroup;
		if (!isConfirmableChallenger(challenger, incumbent)) {
			clearPending();
		} else if (confirmChallenger(challenger)) {
			stickyGroup = challenger;
			lastSeenHubMs = nowHubMs;
			updateFilteredAim(challenger, true);
			clearPending();
			return toTarget(stickyGroup, true, 0L);
		}

		if (currentObservation != null) {
			return toTarget(stickyGroup, true, 0L);
		}
		return null;
	}

	private BallGroup selectCurrentObservation(
			List<BallGroup> freshGroups, boolean enforceDesiredColor) {
		BallGroup best = null;
		double bestDistance = Double.POSITIVE_INFINITY;
		if (freshGroups == null) {
			return null;
		}
		for (BallGroup group : freshGroups) {
			if (!isEligible(group, enforceDesiredColor)
					|| !areAssociated(stickyGroup, group)) {
				continue;
			}
			double distance = angularDistance(stickyGroup, group);
			if (distance < bestDistance
					|| (Double.compare(distance, bestDistance) == 0
							&& BallGrouping.isCloser(group, best))) {
				best = group;
				bestDistance = distance;
			}
		}
		return best;
	}

	private BallGroup selectBestGroup(
			List<BallGroup> freshGroups, BallGroup excluded, boolean enforceDesiredColor) {
		BallGroup best = null;
		if (freshGroups == null) {
			return null;
		}
		for (BallGroup group : freshGroups) {
			if (group == excluded || !isEligible(group, enforceDesiredColor)) {
				continue;
			}
			if (BallGrouping.isHigherPriority(group, best)) {
				best = group;
			}
		}
		return best;
	}

	private boolean hasDesiredColorGroup(List<BallGroup> freshGroups) {
		if (desiredColor == null || freshGroups == null) {
			return false;
		}
		for (BallGroup group : freshGroups) {
			if (isEligible(group, true)) {
				return true;
			}
		}
		return false;
	}

	private boolean isEligible(BallGroup group, boolean enforceDesiredColor) {
		if (group == null) {
			return false;
		}
		if (!enforceDesiredColor) {
			return true;
		}
		if (desiredColor == BallColor.GREEN) {
			return group.getGreenCount() > 0;
		}
		if (desiredColor == BallColor.PURPLE) {
			return group.getPurpleCount() > 0;
		}
		return true;
	}

	private boolean isConfirmableChallenger(BallGroup challenger, BallGroup incumbent) {
		if (challenger == null || incumbent == null) {
			return false;
		}
		if (challenger.getSize() > incumbent.getSize()) {
			return true;
		}
		if (challenger.getSize() != incumbent.getSize()
				|| !BallGrouping.isHigherPriority(challenger, incumbent)) {
			return false;
		}
		double challengerArea = BallGrouping.apparentCloseness(challenger);
		double incumbentArea = BallGrouping.apparentCloseness(incumbent);
		return challengerArea
				> incumbentArea * BallVisionConfig.CLOSEST_AREA_SWITCH_RATIO;
	}

	private boolean confirmChallenger(BallGroup challenger) {
		if (pendingGroup != null && areAssociated(pendingGroup, challenger)) {
			pendingConfirmationFrames++;
		} else {
			pendingConfirmationFrames = 1;
		}
		pendingGroup = challenger;
		return pendingConfirmationFrames >= config.getSwitchConfirmFrames();
	}

	private void clearPending() {
		pendingGroup = null;
		pendingConfirmationFrames = 0;
	}

	private void updateFilteredAim(BallGroup observation, boolean reseed) {
		rawLockedTxDeg = observation.getWeightedTxDeg();
		rawLockedTyDeg = observation.getWeightedTyDeg();
		if (reseed || !hasFilteredAim) {
			filteredTxDeg = rawLockedTxDeg;
			filteredTyDeg = rawLockedTyDeg;
			hasFilteredAim = true;
			return;
		}
		double a = config.getAimFilterGain();
		filteredTxDeg += a * (rawLockedTxDeg - filteredTxDeg);
		filteredTyDeg += a * (rawLockedTyDeg - filteredTyDeg);
	}

	public static boolean areAssociated(BallGroup previous, BallGroup next) {
		if (previous == null || next == null) {
			return false;
		}
		double dTx = previous.getWeightedTxDeg() - next.getWeightedTxDeg();
		double dTy = previous.getWeightedTyDeg() - next.getWeightedTyDeg();
		return Math.hypot(dTx, dTy) <= BallVisionConfig.PERSIST_ASSOCIATION_DEG;
	}

	private static double angularDistance(BallGroup previous, BallGroup next) {
		double dTx = previous.getWeightedTxDeg() - next.getWeightedTxDeg();
		double dTy = previous.getWeightedTyDeg() - next.getWeightedTyDeg();
		return Math.hypot(dTx, dTy);
	}

	private static BallTargetingConfig requireConfig(BallTargetingConfig config) {
		if (config == null) {
			throw new IllegalArgumentException("config must not be null");
		}
		return config;
	}

	public Optional<BallGroup> getStickyGroup() {
		return Optional.ofNullable(stickyGroup);
	}

	public Optional<BallGroup> getPendingGroup() {
		return Optional.ofNullable(pendingGroup);
	}

	public int getPendingConfirmationFrames() {
		return pendingConfirmationFrames;
	}

	public double getRawLockedTxDeg() {
		return rawLockedTxDeg;
	}

	private BallTarget toTarget(BallGroup g, boolean fresh, long ageMs) {
		return new BallTarget(
				true,
				fresh,
				filteredTxDeg,
				filteredTyDeg,
				g.getAverageConfidence(),
				g.getSize(),
				ageMs,
				g.getScore(),
				g.getGreenCount(),
				g.getPurpleCount(),
				desiredColor);
	}
}
