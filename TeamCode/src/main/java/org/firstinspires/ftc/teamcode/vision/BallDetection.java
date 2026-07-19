package org.firstinspires.ftc.teamcode.vision;

import java.util.Collections;
import java.util.List;

/**
 * Immutable project-owned view of one Limelight neural detector result.
 * Bounding-box values are in image pixels when corners are available; otherwise
 * width/height may be estimated from target area.
 */
public final class BallDetection {

	private final String className;
	private final BallColor color;
	private final double confidence;
	private final double txDeg;
	private final double tyDeg;
	private final double targetArea;
	private final List<List<Double>> corners;
	private final double frameTimestamp;
	private final long hubTimestampMs;
	private final double centerXPx;
	private final double centerYPx;
	private final double widthPx;
	private final double heightPx;
	private final boolean hasPixelBox;

	public BallDetection(
			String className,
			double confidence,
			double txDeg,
			double tyDeg,
			double targetArea,
			List<List<Double>> corners,
			double frameTimestamp,
			long hubTimestampMs,
			double centerXPx,
			double centerYPx,
			double widthPx,
			double heightPx,
			boolean hasPixelBox) {
		this.className = className == null ? "" : className;
		this.color = BallColor.fromClassName(this.className);
		this.confidence = confidence;
		this.txDeg = txDeg;
		this.tyDeg = tyDeg;
		this.targetArea = targetArea;
		this.corners = corners == null
				? Collections.<List<Double>>emptyList()
				: Collections.unmodifiableList(corners);
		this.frameTimestamp = frameTimestamp;
		this.hubTimestampMs = hubTimestampMs;
		this.centerXPx = centerXPx;
		this.centerYPx = centerYPx;
		this.widthPx = widthPx;
		this.heightPx = heightPx;
		this.hasPixelBox = hasPixelBox;
	}

	public String getClassName() { return className; }
	public BallColor getColor() { return color; }
	public double getConfidence() { return confidence; }
	public double getTxDeg() { return txDeg; }
	public double getTyDeg() { return tyDeg; }
	public double getTargetArea() { return targetArea; }
	public List<List<Double>> getCorners() { return corners; }
	public double getFrameTimestamp() { return frameTimestamp; }
	public long getHubTimestampMs() { return hubTimestampMs; }
	public double getCenterXPx() { return centerXPx; }
	public double getCenterYPx() { return centerYPx; }
	public double getWidthPx() { return widthPx; }
	public double getHeightPx() { return heightPx; }
	public boolean hasPixelBox() { return hasPixelBox; }
}
