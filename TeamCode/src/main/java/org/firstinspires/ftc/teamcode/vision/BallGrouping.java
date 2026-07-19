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

	public static List<BallDetection> filterDetections(List<BallDetection> raw) {
		if (raw == null || raw.isEmpty()) {
			return Collections.emptyList();
		}
		List<BallDetection> out = new ArrayList<>();
		for (BallDetection d : raw) {
			if (d == null) {
				continue;
			}
			if (d.getConfidence() < BallVisionConfig.MIN_CONFIDENCE) {
				continue;
			}
			if (d.getTargetArea() < BallVisionConfig.MIN_TARGET_AREA) {
				continue;
			}
			if (d.getColor() != BallColor.GREEN && d.getColor() != BallColor.PURPLE) {
				continue;
			}
			if (isNearImageEdge(d)) {
				continue;
			}
			out.add(d);
		}
		return out;
	}

	public static boolean isNearImageEdge(BallDetection d) {
		double marginX = BallVisionConfig.IMAGE_WIDTH_PX * BallVisionConfig.EDGE_MARGIN_FRACTION;
		double marginY = BallVisionConfig.IMAGE_HEIGHT_PX * BallVisionConfig.EDGE_MARGIN_FRACTION;
		if (d.hasPixelBox()) {
			double left = d.getCenterXPx() - d.getWidthPx() / 2.0;
			double right = d.getCenterXPx() + d.getWidthPx() / 2.0;
			double top = d.getCenterYPx() - d.getHeightPx() / 2.0;
			double bottom = d.getCenterYPx() + d.getHeightPx() / 2.0;
			return left < marginX
					|| right > BallVisionConfig.IMAGE_WIDTH_PX - marginX
					|| top < marginY
					|| bottom > BallVisionConfig.IMAGE_HEIGHT_PX - marginY;
		}
		// Angular fallback: treat large |tx|/|ty| as near FOV edge (~±30° typical).
		return Math.abs(d.getTxDeg()) > 25.0 || Math.abs(d.getTyDeg()) > 20.0;
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

	/** Deterministic BFS connected-components grouping. */
	public static List<BallGroup> groupDetections(List<BallDetection> detections) {
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
			while (!queue.isEmpty()) {
				int cur = queue.removeFirst();
				members.add(detections.get(cur));
				for (int j = 0; j < n; j++) {
					if (!visited[j] && areNearby(detections.get(cur), detections.get(j))) {
						visited[j] = true;
						queue.add(j);
					}
				}
			}
			groups.add(buildGroup(members, null));
		}
		return groups;
	}

	public static BallGroup buildGroup(List<BallDetection> members, BallColor desiredColor) {
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
		double score = scoreGroup(avgTx, avgTy, size, sumArea, avgConf, green, purple,
				anyBox, minX, maxX, minY, maxY, desiredColor);

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
			boolean hasBox,
			double minX,
			double maxX,
			double minY,
			double maxY,
			BallColor desiredColor) {
		double score = 0.0;
		double centerline = 1.0 - Math.min(1.0, Math.abs(avgTx) / BallVisionConfig.CENTERLINE_SOFT_DEG);
		score += BallVisionConfig.SCORE_CENTERLINE * centerline;
		score += BallVisionConfig.SCORE_SIZE * Math.min(1.0, size / 3.0);
		score += BallVisionConfig.SCORE_AREA * Math.min(1.0, totalArea / 10.0);
		score += BallVisionConfig.SCORE_CONFIDENCE * Math.min(1.0, avgConf / 100.0);

		if (desiredColor == BallColor.GREEN || desiredColor == BallColor.PURPLE) {
			int desired = desiredColor == BallColor.GREEN ? green : purple;
			int other = desiredColor == BallColor.GREEN ? purple : green;
			score += BallVisionConfig.SCORE_DESIRED_COLOR * Math.min(1.0, desired / 2.0);
			if (desired == 0 && other > 0) {
				score -= BallVisionConfig.PENALTY_WRONG_COLOR;
			}
		}

		if (avgConf < BallVisionConfig.MIN_CONFIDENCE + 10.0) {
			score -= BallVisionConfig.PENALTY_LOW_CONFIDENCE;
		}

		if (hasBox) {
			double marginX = BallVisionConfig.IMAGE_WIDTH_PX * BallVisionConfig.EDGE_MARGIN_FRACTION * 2.0;
			double marginY = BallVisionConfig.IMAGE_HEIGHT_PX * BallVisionConfig.EDGE_MARGIN_FRACTION * 2.0;
			if (minX < marginX || maxX > BallVisionConfig.IMAGE_WIDTH_PX - marginX
					|| minY < marginY || maxY > BallVisionConfig.IMAGE_HEIGHT_PX - marginY) {
				score -= BallVisionConfig.PENALTY_EDGE;
			}
		} else if (Math.abs(avgTx) > 20.0 || Math.abs(avgTy) > 15.0) {
			score -= BallVisionConfig.PENALTY_EDGE;
		}

		return score;
	}

	public static Optional<BallGroup> selectBestGroup(List<BallGroup> groups, BallColor desiredColor) {
		if (groups == null || groups.isEmpty()) {
			return Optional.empty();
		}
		BallGroup best = null;
		double bestScore = Double.NEGATIVE_INFINITY;
		for (BallGroup g : groups) {
			BallGroup scored = desiredColor == null
					? g
					: buildGroup(g.getMembers(), desiredColor);
			if (scored.getScore() > bestScore) {
				bestScore = scored.getScore();
				best = scored;
			}
		}
		return Optional.ofNullable(best);
	}
}
