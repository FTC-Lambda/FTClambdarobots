package org.firstinspires.ftc.teamcode.vision.hive;

/** Pure geometry helpers for turning Limelight AprilTag data into a tag height above the floor. */
public final class HiveGeometry {
	private static final double ZERO_EPS = 1e-6;

	private HiveGeometry() {}

	/**
	 * Straight-line distance from the camera to the tag, in the same units as the inputs (inches).
	 * Inputs are the tag position in the camera frame. Returns NaN if any input is non-finite or
	 * the vector is ~0 (the Limelight reports an all-zero pose when its 3D solve failed).
	 */
	public static double rangeInches(double xIn, double yIn, double zIn) {
		if (!isFinite(xIn) || !isFinite(yIn) || !isFinite(zIn)) return Double.NaN;
		double r = Math.sqrt(xIn * xIn + yIn * yIn + zIn * zIn);
		if (r < ZERO_EPS) return Double.NaN;
		return r;
	}

	/**
	 * Height of a tag above the floor, in inches.
	 * <p>
	 * txDeg/tyDeg are the Limelight pinhole angles in degrees (ty positive above the crosshair).
	 * The camera-frame ray (x right, y up, z forward) is (tan tx, tan ty, 1), normalized. The
	 * camera is cameraHeightIn above the floor and tilted cameraPitchDeg upward from horizontal
	 * (positive = looking up). Height = cameraHeightIn + rangeIn * (world-up part of the ray).
	 * Returns NaN if any input is non-finite or rangeIn is not positive.
	 */
	public static double tagHeightInches(double rangeIn, double txDeg, double tyDeg,
	                                     double cameraHeightIn, double cameraPitchDeg) {
		if (!isFinite(rangeIn) || !isFinite(txDeg) || !isFinite(tyDeg)
				|| !isFinite(cameraHeightIn) || !isFinite(cameraPitchDeg)) {
			return Double.NaN;
		}
		if (rangeIn <= 0.0) return Double.NaN;
		double tanTx = Math.tan(Math.toRadians(txDeg));
		double tanTy = Math.tan(Math.toRadians(tyDeg));
		double pitch = Math.toRadians(cameraPitchDeg);
		double norm = Math.sqrt(tanTx * tanTx + tanTy * tanTy + 1.0);
		double up = (tanTy * Math.cos(pitch) + Math.sin(pitch)) / norm;
		return cameraHeightIn + rangeIn * up;
	}

	private static boolean isFinite(double v) {
		return !Double.isNaN(v) && !Double.isInfinite(v);
	}
}
