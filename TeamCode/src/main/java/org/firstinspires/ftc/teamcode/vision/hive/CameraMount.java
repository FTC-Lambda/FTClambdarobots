package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Constants;

/**
 * Where the Limelight sits on the robot. Robot frame: x forward, y left, z up, with the origin
 * on the floor under the robot's center (the point whose field position we estimate).
 * The camera is assumed level side to side (no roll).
 */
public final class CameraMount {
	/** Lens position ahead of robot center, inches. */
	public final double forwardIn;
	/** Lens position left of robot center, inches. */
	public final double leftIn;
	/** Lens height above the tiles, inches. */
	public final double heightIn;
	/** Camera yaw from robot forward, degrees, counter-clockwise (left) positive. */
	public final double yawDeg;
	/** Camera upward tilt from horizontal, degrees (positive = looking up). */
	public final double pitchDeg;

	public CameraMount(double forwardIn, double leftIn, double heightIn, double yawDeg, double pitchDeg) {
		this.forwardIn = forwardIn;
		this.leftIn = leftIn;
		this.heightIn = heightIn;
		this.yawDeg = yawDeg;
		this.pitchDeg = pitchDeg;
	}

	/** Mount from {@link Constants} (LIMELIGHT_FORWARD_IN, _LEFT_IN, _HEIGHT_IN, _YAW_DEG, _PITCH_DEG). */
	public static CameraMount fromConstants() {
		return new CameraMount(Constants.LIMELIGHT_FORWARD_IN, Constants.LIMELIGHT_LEFT_IN,
				Constants.LIMELIGHT_HEIGHT_IN, Constants.LIMELIGHT_YAW_DEG, Constants.LIMELIGHT_PITCH_DEG);
	}

	public CameraMount withHeightAndPitch(double heightIn, double pitchDeg) {
		return new CameraMount(forwardIn, leftIn, heightIn, yawDeg, pitchDeg);
	}

	/**
	 * Tag position in the ROBOT frame from a Limelight reading. Uses the 3D range plus the
	 * crosshair-free pinhole angles (ray = (tan tx, tan ty, 1) normalized, tx right-positive,
	 * ty up-positive), so it does not depend on the Limelight's pose-axis conventions.
	 * z is the tag's height above the tiles and matches {@link HiveGeometry#tagHeightInches}.
	 * Returns null if any input is non-finite or the range is not positive.
	 */
	public Vec3 toRobotFrame(double rangeIn, double txDeg, double tyDeg) {
		if (!finite(rangeIn) || !finite(txDeg) || !finite(tyDeg) || rangeIn <= 0.0) {
			return null;
		}
		double tanX = Math.tan(Math.toRadians(txDeg));
		double tanY = Math.tan(Math.toRadians(tyDeg));
		double n = Math.sqrt(tanX * tanX + tanY * tanY + 1.0);
		double right = rangeIn * tanX / n;
		double up = rangeIn * tanY / n;
		double fwd = rangeIn / n;

		double p = Math.toRadians(pitchDeg);
		double fwdLevel = fwd * Math.cos(p) - up * Math.sin(p);
		double upLevel = fwd * Math.sin(p) + up * Math.cos(p);
		double left = -right;

		double yaw = Math.toRadians(yawDeg);
		double x = fwdLevel * Math.cos(yaw) - left * Math.sin(yaw);
		double y = fwdLevel * Math.sin(yaw) + left * Math.cos(yaw);
		return new Vec3(forwardIn + x, leftIn + y, heightIn + upLevel);
	}

	private static boolean finite(double v) {
		return !Double.isNaN(v) && !Double.isInfinite(v);
	}
}
