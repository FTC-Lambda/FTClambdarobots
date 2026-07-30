package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/**
 * Pure functions for filtering, proximity, BFS grouping, scoring, and selection.
 * Unit-testable with no Limelight / Android dependencies.
 */
public final class BallGrouping {

	private BallGrouping() {}

	/** Per-frame tally of why detections were dropped, for driver-station diagnosis. */
	public static final class RejectCounts {
		public int lowConfidence;
		public int smallArea;
		public int unknownColor;
		public int nearEdge;

		public void reset() {
			lowConfidence = 0;
			smallArea = 0;
			unknownColor = 0;
			nearEdge = 0;
		}

		public int total() {
			return lowConfidence + smallArea + unknownColor + nearEdge;
		}

		@Override
		public String toString() {
			return "conf=" + lowConfidence + " area=" + smallArea
					+ " color=" + unknownColor + " edge=" + nearEdge;
		}
	}

	public static List<BallDetection> filterDetections(List<BallDetection> raw) {
		return filterDetections(raw, null);
	}

	public static List<BallDetection> filterDetections(List<BallDetection> raw, RejectCounts counts) {
		if (counts != null) {
			counts.reset();
		}
		if (raw == null || raw.isEmpty()) {
			return Collections.emptyList();
		}
		List<BallDetection> out = new ArrayList<>();
		for (BallDetection d : raw) {
			if (d == null) {
				continue;
			}
			if (d.getConfidence() < BallVisionConfig.MIN_CONFIDENCE) {
				if (counts != null) {
					counts.lowConfidence++;
				}
				continue;
			}
			if (d.getTargetArea() < BallVisionConfig.MIN_TARGET_AREA) {
				if (counts != null) {
					counts.smallArea++;
				}
				continue;
			}
			if (d.getColor() != BallColor.GREEN && d.getColor() != BallColor.PURPLE) {
				if (counts != null) {
					counts.unknownColor++;
				}
				continue;
			}
			if (isNearImageEdge(d)) {
				if (counts != null) {
					counts.nearEdge++;
				}
				continue;
			}
			out.add(d);
		}
		return out;
	}

	/**
	 * Angular edge test. Deliberately not pixel-based: corner coordinates arrive in
	 * whatever resolution the pipeline captures at, so a hardcoded frame size would
	 * silently reject valid detections whenever that resolution changed.
	 */
	public static boolean isNearImageEdge(BallDetection d) {
		return isNearAngularEdge(d.getTxDeg(), d.getTyDeg(), 0.0);
	}

	static boolean isNearAngularEdge(double txDeg, double tyDeg, double marginDeg) {
		return Math.abs(txDeg) > BallVisionConfig.EDGE_MAX_TX_DEG - marginDeg
				|| tyDeg > BallVisionConfig.EDGE_MAX_TY_DEG - marginDeg
				|| tyDeg < BallVisionConfig.EDGE_MIN_TY_DEG + marginDeg;
	}

	/**
	 * True when two detections are spatially adjacent.
	 * Preferred: bounding-box gap scaled by average width/height.
	 * Fallback: tx/ty angular thresholds when no pixel box is available.
	 */
	public static boolean areNearby(BallDetection a, BallDetection b) {
		if (a == null || b == null) {
			return false;
		}
		if (a.hasPixelBox() && b.hasPixelBox()) {
			double gapX = Math.max(0.0,
					Math.abs(a.getCenterXPx() - b.getCenterXPx())
							- (a.getWidthPx() + b.getWidthPx()) / 2.0);
			double gapY = Math.max(0.0,
					Math.abs(a.getCenterYPx() - b.getCenterYPx())
							- (a.getHeightPx() + b.getHeightPx()) / 2.0);
			double avgW = Math.max(1.0, (a.getWidthPx() + b.getWidthPx()) / 2.0);
			double avgH = Math.max(1.0, (a.getHeightPx() + b.getHeightPx()) / 2.0);
			return gapX <= BallVisionConfig.MAX_HORIZONTAL_GAP_IN_WIDTHS * avgW
					&& gapY <= BallVisionConfig.MAX_VERTICAL_GAP_IN_HEIGHTS * avgH;
		}
		return Math.abs(a.getTxDeg() - b.getTxDeg()) <= BallVisionConfig.MAX_TX_DIFF_DEG
				&& Math.abs(a.getTyDeg() - b.getTyDeg()) <= BallVisionConfig.MAX_TY_DIFF_DEG;
	}

	public static List<BallGroup> groupDetections(List<BallDetection> detections) {
		return groupDetections(detections, null);
	}

	/** Deterministic BFS connected-components grouping, capped by group angular span. */
	public static List<BallGroup> groupDetections(
			List<BallDetection> detections, BallColor desiredColor) {
		if (detections == null || detections.isEmpty()) {
			return Collections.emptyList();
		}
		int n = detections.size();
		boolean[] visited = new boolean[n];
		List<BallGroup> groups = new ArrayList<>();

		for (int i = 0; i < n; i++) {
			if (visited[i]) {
				continue;
			}
			List<BallDetection> members = new ArrayList<>();
			ArrayDeque<Integer> queue = new ArrayDeque<>();
			queue.add(i);
			visited[i] = true;
			double minTx = detections.get(i).getTxDeg();
			double maxTx = minTx;
			while (!queue.isEmpty()) {
				int cur = queue.removeFirst();
				members.add(detections.get(cur));
				for (int j = 0; j < n; j++) {
					if (visited[j]) {
						continue;
					}
					BallDetection candidate = detections.get(j);
					if (!areNearby(detections.get(cur), candidate)) {
						continue;
					}
					double nextMin = Math.min(minTx, candidate.getTxDeg());
					double nextMax = Math.max(maxTx, candidate.getTxDeg());
					if (nextMax - nextMin > BallVisionConfig.MAX_GROUP_SPAN_DEG) {
						continue;
					}
					minTx = nextMin;
					maxTx = nextMax;
					visited[j] = true;
					queue.add(j);
				}
			}
			groups.add(buildGroup(members, desiredColor));
		}
		return groups;
	}

	public static BallGroup buildGroup(List<BallDetection> members, BallColor desiredColor) {
		if (members == null || members.isEmpty()) {
			throw new IllegalArgumentException("buildGroup requires at least one member");
		}
		List<BallDetection> copy = new ArrayList<>(members);
		copy.sort(Comparator
				.comparingDouble(BallDetection::getTxDeg)
				.thenComparingDouble(BallDetection::getTyDeg)
				.thenComparing(BallDetection::getClassName));

		double sumTx = 0, sumTy = 0, sumArea = 0, sumConf = 0;
		double wSumTx = 0, wSumTy = 0, wSum = 0;
		int green = 0, purple = 0;
		BallDetection largest = copy.get(0);
		double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
		double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
		boolean anyBox = false;

		for (BallDetection d : copy) {
			sumTx += d.getTxDeg();
			sumTy += d.getTyDeg();
			sumArea += d.getTargetArea();
			sumConf += d.getConfidence();
			double w = Math.max(1e-3, d.getConfidence() * Math.max(1e-3, d.getTargetArea()));
			wSumTx += d.getTxDeg() * w;
			wSumTy += d.getTyDeg() * w;
			wSum += w;
			if (d.getColor() == BallColor.GREEN) {
				green++;
			} else if (d.getColor() == BallColor.PURPLE) {
				purple++;
			}
			if (d.getTargetArea() > largest.getTargetArea()) {
				largest = d;
			}
			if (d.hasPixelBox()) {
				anyBox = true;
				minX = Math.min(minX, d.getCenterXPx() - d.getWidthPx() / 2.0);
				maxX = Math.max(maxX, d.getCenterXPx() + d.getWidthPx() / 2.0);
				minY = Math.min(minY, d.getCenterYPx() - d.getHeightPx() / 2.0);
				maxY = Math.max(maxY, d.getCenterYPx() + d.getHeightPx() / 2.0);
			}
		}

		int size = copy.size();
		double avgTx = sumTx / size;
		double avgTy = sumTy / size;
		double weightedTx = wSumTx / wSum;
		double weightedTy = wSumTy / wSum;
		double avgArea = sumArea / size;
		double avgConf = sumConf / size;
		double score = scoreGroup(avgTx, avgTy, size, sumArea, avgConf, green, purple, desiredColor);

		return new BallGroup(
				copy, avgTx, avgTy, weightedTx, weightedTy,
				sumArea, avgArea, avgConf, green, purple, largest,
				anyBox ? minX : 0, anyBox ? minY : 0, anyBox ? maxX : 0, anyBox ? maxY : 0,
				anyBox, score);
	}

	public static double scoreGroup(
			double avgTx,
			double avgTy,
			int size,
			double totalArea,
			double avgConf,
			int green,
			int purple,
			BallColor desiredColor) {
		double score = 0.0;
		double centerline = 1.0 - Math.min(1.0, Math.abs(avgTx) / BallVisionConfig.CENTERLINE_SOFT_DEG);
		score += BallVisionConfig.SCORE_CENTERLINE * centerline;
		score += BallVisionConfig.SCORE_SIZE
				* Math.min(1.0, (double) size / BallVisionConfig.SCORE_SIZE_SATURATION);
		score += BallVisionConfig.SCORE_AREA
				* Math.min(1.0, totalArea / BallVisionConfig.SCORE_AREA_SATURATION);
		score += BallVisionConfig.SCORE_CONFIDENCE * Math.min(1.0, avgConf);

		if (desiredColor == BallColor.GREEN || desiredColor == BallColor.PURPLE) {
			int desired = desiredColor == BallColor.GREEN ? green : purple;
			int other = desiredColor == BallColor.GREEN ? purple : green;
			score += BallVisionConfig.SCORE_DESIRED_COLOR * Math.min(1.0, desired / 2.0);
			if (other > 0) {
				// Scaled by the mismatched fraction, not all-or-nothing: the aimpoint is the
				// group mean, so even one wrong-colour member drags the robot off target.
				score -= BallVisionConfig.PENALTY_WRONG_COLOR
						* ((double) other / (desired + other));
			}
		}

		if (avgConf < BallVisionConfig.MIN_CONFIDENCE + BallVisionConfig.LOW_CONFIDENCE_MARGIN) {
			score -= BallVisionConfig.PENALTY_LOW_CONFIDENCE;
		}

		if (isNearAngularEdge(avgTx, avgTy, BallVisionConfig.EDGE_PENALTY_MARGIN_DEG)) {
			score -= BallVisionConfig.PENALTY_EDGE;
		}

		return score;
	}

	/**
	 * Apparent closeness for same-size DECODE balls: larger max member {@code ta}
	 * means nearer the camera. Not true range in inches.
	 */
	public static double apparentCloseness(BallGroup g) {
		if (g == null) {
			return 0.0;
		}
		BallDetection largest = g.getClosestOrLargest();
		if (largest != null) {
			return largest.getTargetArea();
		}
		return g.getTotalArea();
	}

	/**
	 * True when {@code a} should beat {@code b} as the closer group.
	 * Primary key is max member area; ties break on confidence, then smaller |tx|.
	 */
	public static boolean isCloser(BallGroup a, BallGroup b) {
		if (a == null) {
			return false;
		}
		if (b == null) {
			return true;
		}
		double da = apparentCloseness(a);
		double db = apparentCloseness(b);
		if (da != db) {
			return da > db;
		}
		if (a.getAverageConfidence() != b.getAverageConfidence()) {
			return a.getAverageConfidence() > b.getAverageConfidence();
		}
		return Math.abs(a.getWeightedTxDeg()) < Math.abs(b.getWeightedTxDeg());
	}

	/** True when {@code candidate} should beat {@code incumbent} for ball acquisition. */
	public static boolean isHigherPriority(BallGroup candidate, BallGroup incumbent) {
		if (candidate == null) {
			return false;
		}
		if (incumbent == null) {
			return true;
		}
		if (candidate.getSize() != incumbent.getSize()) {
			return candidate.getSize() > incumbent.getSize();
		}
		int areaCmp = Double.compare(apparentCloseness(candidate), apparentCloseness(incumbent));
		if (areaCmp != 0) {
			return areaCmp > 0;
		}
		int confCmp = Double.compare(candidate.getAverageConfidence(), incumbent.getAverageConfidence());
		if (confCmp != 0) {
			return confCmp > 0;
		}
		return Math.abs(candidate.getWeightedTxDeg()) < Math.abs(incumbent.getWeightedTxDeg());
	}

	/** Selects a group by ball count, then apparent closeness; optional colour preference. */
	public static Optional<BallGroup> selectBestGroup(List<BallGroup> groups) {
		return selectPriorityGroup(groups, null);
	}

	/** Selects a priority group, preferring groups that contain {@code desiredColor}. */
	public static Optional<BallGroup> selectBestGroup(List<BallGroup> groups, BallColor desiredColor) {
		return selectPriorityGroup(groups, desiredColor);
	}

	public static Optional<BallGroup> selectClosestGroup(List<BallGroup> groups, BallColor desiredColor) {
		BallGroup best = null;
		for (BallGroup g : eligibleGroups(groups, desiredColor)) {
			if (isCloser(g, best)) {
				best = g;
			}
		}
		return Optional.ofNullable(best);
	}

	public static Optional<BallGroup> selectPriorityGroup(List<BallGroup> groups, BallColor desiredColor) {
		BallGroup best = null;
		for (BallGroup g : eligibleGroups(groups, desiredColor)) {
			if (isHigherPriority(g, best)) {
				best = g;
			}
		}
		return Optional.ofNullable(best);
	}

	private static List<BallGroup> eligibleGroups(List<BallGroup> groups, BallColor desiredColor) {
		if (groups == null || groups.isEmpty()) {
			return Collections.emptyList();
		}
		List<BallGroup> candidates = groups;
		if (desiredColor == BallColor.GREEN || desiredColor == BallColor.PURPLE) {
			List<BallGroup> matching = new ArrayList<>();
			for (BallGroup g : groups) {
				if (g == null) {
					continue;
				}
				int count = desiredColor == BallColor.GREEN ? g.getGreenCount() : g.getPurpleCount();
				if (count > 0) {
					matching.add(g);
				}
			}
			if (!matching.isEmpty()) {
				candidates = matching;
			}
		}
		return candidates;
	}
}
