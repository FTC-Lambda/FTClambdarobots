package org.firstinspires.ftc.teamcode.vision;

/**
 * Control-facing ball target snapshot. Does not command motors.
 */
public final class BallTarget {

	private final boolean valid;
	private final boolean fresh;
	private final double horizontalErrorDeg;
	private final double verticalAngleDeg;
	private final double confidence;
	private final int groupSize;
	private final long ageMs;
	private final double score;
	private final int greenCount;
	private final int purpleCount;
	private final BallColor desiredColor;

	public static BallTarget none() {
		return new BallTarget(false, false, 0, 0, 0, 0, Long.MAX_VALUE, 0, 0, 0, null);
	}

	public BallTarget(
			boolean valid,
			boolean fresh,
			double horizontalErrorDeg,
			double verticalAngleDeg,
			double confidence,
			int groupSize,
			long ageMs,
			double score,
			int greenCount,
			int purpleCount,
			BallColor desiredColor) {
		this.valid = valid;
		this.fresh = fresh;
		this.horizontalErrorDeg = horizontalErrorDeg;
		this.verticalAngleDeg = verticalAngleDeg;
		this.confidence = confidence;
		this.groupSize = groupSize;
		this.ageMs = ageMs;
		this.score = score;
		this.greenCount = greenCount;
		this.purpleCount = purpleCount;
		this.desiredColor = desiredColor;
	}

	public boolean isValid() { return valid; }
	public boolean isFresh() { return fresh; }
	public double getHorizontalErrorDeg() { return horizontalErrorDeg; }
	public double getVerticalAngleDeg() { return verticalAngleDeg; }
	public double getConfidence() { return confidence; }
	public int getGroupSize() { return groupSize; }
	public long getAgeMs() { return ageMs; }
	public double getScore() { return score; }
	public int getGreenCount() { return greenCount; }
	public int getPurpleCount() { return purpleCount; }
	public BallColor getDesiredColor() { return desiredColor; }
}
