package org.firstinspires.ftc.teamcode.vision;

import java.util.Collections;
import java.util.List;

/**
 * Immutable connected group of nearby ball detections.
 * {@code totalArea} / {@code averageArea} are apparent image-size heuristics, not meters.
 */
public final class BallGroup {

	private final List<BallDetection> members;
	private final double averageTxDeg;
	private final double averageTyDeg;
	private final double weightedTxDeg;
	private final double weightedTyDeg;
	private final double totalArea;
	private final double averageArea;
	private final double averageConfidence;
	private final int greenCount;
	private final int purpleCount;
	private final BallDetection closestOrLargest;
	private final double boxMinX;
	private final double boxMinY;
	private final double boxMaxX;
	private final double boxMaxY;
	private final boolean hasGroupBox;
	private final double score;

	public BallGroup(
			List<BallDetection> members,
			double averageTxDeg,
			double averageTyDeg,
			double weightedTxDeg,
			double weightedTyDeg,
			double totalArea,
			double averageArea,
			double averageConfidence,
			int greenCount,
			int purpleCount,
			BallDetection closestOrLargest,
			double boxMinX,
			double boxMinY,
			double boxMaxX,
			double boxMaxY,
			boolean hasGroupBox,
			double score) {
		this.members = Collections.unmodifiableList(members);
		this.averageTxDeg = averageTxDeg;
		this.averageTyDeg = averageTyDeg;
		this.weightedTxDeg = weightedTxDeg;
		this.weightedTyDeg = weightedTyDeg;
		this.totalArea = totalArea;
		this.averageArea = averageArea;
		this.averageConfidence = averageConfidence;
		this.greenCount = greenCount;
		this.purpleCount = purpleCount;
		this.closestOrLargest = closestOrLargest;
		this.boxMinX = boxMinX;
		this.boxMinY = boxMinY;
		this.boxMaxX = boxMaxX;
		this.boxMaxY = boxMaxY;
		this.hasGroupBox = hasGroupBox;
		this.score = score;
	}

	public List<BallDetection> getMembers() { return members; }
	public int getSize() { return members.size(); }
	public double getAverageTxDeg() { return averageTxDeg; }
	public double getAverageTyDeg() { return averageTyDeg; }
	public double getWeightedTxDeg() { return weightedTxDeg; }
	public double getWeightedTyDeg() { return weightedTyDeg; }
	public double getTotalArea() { return totalArea; }
	public double getAverageArea() { return averageArea; }
	public double getAverageConfidence() { return averageConfidence; }
	public int getGreenCount() { return greenCount; }
	public int getPurpleCount() { return purpleCount; }
	public BallDetection getClosestOrLargest() { return closestOrLargest; }
	public double getBoxMinX() { return boxMinX; }
	public double getBoxMinY() { return boxMinY; }
	public double getBoxMaxX() { return boxMaxX; }
	public double getBoxMaxY() { return boxMaxY; }
	public boolean hasGroupBox() { return hasGroupBox; }
	public double getScore() { return score; }
}
