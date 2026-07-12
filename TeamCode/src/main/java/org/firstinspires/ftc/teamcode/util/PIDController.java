package org.firstinspires.ftc.teamcode.util;

/**
 * PID controller for vision tracking loops (bearing-to-tag, range-to-tag).
 *
 * Derivative is computed on the raw measurement rather than the error, so a setpoint change
 * (switching target tag, adjusting standoff distance) doesn't cause a derivative "kick" — only
 * an actual change in the sensor reading does. That raw derivative is then run through a
 * low-pass filter, since differentiating a noisy Limelight reading frame-to-frame amplifies
 * the noise; the filtered term also doubles as feedforward "lead" on a moving tag (the same
 * role the hand-rolled bearingRate/rangeRate estimation used to play).
 *
 * Integral only accumulates inside integralZone (small, persistent errors — e.g. a camera
 * mounting offset) so it doesn't wind up while still far from the setpoint, and is separately
 * clamped to integralLimit as a second anti-windup guard.
 */
public class PIDController {

	private final double kP, kI, kD;
	private final double integralLimit;
	private final double integralZone;
	private final double derivativeFilterGain; // 0..1: higher = snappier/less filtered

	private double integralSum;
	private double filteredDerivative;
	private double prevMeasurement;
	private boolean hasPrevMeasurement;

	private double lastP, lastI, lastD;

	public PIDController(double kP, double kI, double kD, double integralLimit,
						  double integralZone, double derivativeFilterGain) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.integralLimit = integralLimit;
		this.integralZone = integralZone;
		this.derivativeFilterGain = derivativeFilterGain;
	}

	/**
	 * @param measurement current sensor reading (e.g. bearing in degrees, range in inches)
	 * @param setpoint    desired value (0 for centering, standoff distance for range-hold, ...)
	 * @param dtSeconds   time since the previous call on this instance
	 * @return P + I + D control output, in the same "units" as the caller's gains produce
	 */
	public double calculate(double measurement, double setpoint, double dtSeconds) {
		double error = measurement - setpoint;

		if (Math.abs(error) > integralZone) {
			integralSum = 0; // still far from the setpoint: don't wind up while catching up
		} else if (dtSeconds > 1e-4) {
			integralSum = clamp(integralSum + error * dtSeconds, -integralLimit, integralLimit);
		}

		double rawDerivative = 0;
		if (hasPrevMeasurement && dtSeconds > 1e-4) {
			rawDerivative = (measurement - prevMeasurement) / dtSeconds;
		}
		filteredDerivative += derivativeFilterGain * (rawDerivative - filteredDerivative);
		prevMeasurement = measurement;
		hasPrevMeasurement = true;

		lastP = kP * error;
		lastI = kI * integralSum;
		lastD = kD * filteredDerivative;

		return lastP + lastI + lastD;
	}

	/** Clears integral, derivative filter, and the previous-measurement history. */
	public void reset() {
		integralSum = 0;
		filteredDerivative = 0;
		hasPrevMeasurement = false;
		lastP = lastI = lastD = 0;
	}

	public double getLastP() { return lastP; }
	public double getLastI() { return lastI; }
	public double getLastD() { return lastD; }

	private static double clamp(double v, double lo, double hi) {
		return Math.max(lo, Math.min(hi, v));
	}
}
