package org.firstinspires.ftc.teamcode.util;

/**
 * Caps how fast a commanded value (e.g. drive/turn power) can change per second, so PID
 * corrections and measurement noise spikes ramp into motors instead of snapping — smoother
 * motion and less wheel slip, at the cost of a small amount of responsiveness.
 */
public class SlewRateLimiter {

	private final double maxChangePerSecond;
	private double prevValue;
	private boolean hasPrev;

	public SlewRateLimiter(double maxChangePerSecond) {
		this.maxChangePerSecond = maxChangePerSecond;
	}

	public double calculate(double target, double dtSeconds) {
		if (!hasPrev || dtSeconds <= 1e-4) {
			prevValue = target;
			hasPrev = true;
			return target;
		}
		double maxDelta = maxChangePerSecond * dtSeconds;
		double delta = clamp(target - prevValue, -maxDelta, maxDelta);
		prevValue += delta;
		return prevValue;
	}

	/** Next calculate() call will jump straight to its target instead of ramping from the old value. */
	public void reset() {
		hasPrev = false;
		prevValue = 0;
	}

	private static double clamp(double v, double lo, double hi) {
		return Math.max(lo, Math.min(hi, v));
	}
}
