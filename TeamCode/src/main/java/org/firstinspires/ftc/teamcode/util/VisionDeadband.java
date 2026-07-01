package org.firstinspires.ftc.teamcode.util;

/**
 * Hysteresis deadbands for vision-based drive correction. Reduces motor stutter when
 * bearing/distance readings jitter near the setpoint.
 */
public class VisionDeadband {

	private double bearingDeadbandDeg;
	private double distanceDeadbandIn;
	private boolean turnActive;
	private boolean driveActive;

	public VisionDeadband() {
		resetToDefaults();
	}

	public void resetToDefaults() {
		bearingDeadbandDeg = Constants.VISION_BEARING_DEADBAND_DEG;
		distanceDeadbandIn = Constants.VISION_DISTANCE_DEADBAND_IN;
		turnActive = false;
		driveActive = false;
	}

	/** True when turn correction should run (outside hysteresis band). */
	public boolean shouldCorrectTurn(double bearingDeg) {
		double abs = Math.abs(bearingDeg);
		double startAt = bearingDeadbandDeg + Constants.VISION_BEARING_HYSTERESIS_START_OFFSET;
		double stopAt = Math.max(0.5, bearingDeadbandDeg - Constants.VISION_BEARING_HYSTERESIS_STOP_OFFSET);
		if (!turnActive) {
			if (abs > startAt) turnActive = true;
		} else if (abs < stopAt) {
			turnActive = false;
		}
		return turnActive;
	}

	/** True when forward/back correction should run (outside hysteresis band). */
	public boolean shouldCorrectDrive(double distErrorIn) {
		double abs = Math.abs(distErrorIn);
		double startAt = distanceDeadbandIn + Constants.VISION_DISTANCE_HYSTERESIS_START_OFFSET;
		double stopAt = Math.max(0.25, distanceDeadbandIn - Constants.VISION_DISTANCE_HYSTERESIS_STOP_OFFSET);
		if (!driveActive) {
			if (abs > startAt) driveActive = true;
		} else if (abs < stopAt) {
			driveActive = false;
		}
		return driveActive;
	}

	/** True when bearing is close enough to count as centered (for phase transitions). */
	public boolean isBearingCentered(double bearingDeg) {
		return Math.abs(bearingDeg) <= Math.max(0.5,
				bearingDeadbandDeg - Constants.VISION_BEARING_HYSTERESIS_STOP_OFFSET);
	}

	public void nudgeBearingDeadband(double deltaDeg) {
		bearingDeadbandDeg = clamp(
				bearingDeadbandDeg + deltaDeg,
				Constants.VISION_BEARING_DEADBAND_MIN,
				Constants.VISION_BEARING_DEADBAND_MAX);
	}

	public void nudgeDistanceDeadband(double deltaIn) {
		distanceDeadbandIn = clamp(
				distanceDeadbandIn + deltaIn,
				Constants.VISION_DISTANCE_DEADBAND_MIN,
				Constants.VISION_DISTANCE_DEADBAND_MAX);
	}

	public double getBearingDeadbandDeg() {
		return bearingDeadbandDeg;
	}

	public double getDistanceDeadbandIn() {
		return distanceDeadbandIn;
	}

	public void clearState() {
		turnActive = false;
		driveActive = false;
	}

	private static double clamp(double v, double lo, double hi) {
		return Math.max(lo, Math.min(hi, v));
	}
}
