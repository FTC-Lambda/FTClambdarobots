package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.Constants;

/** Immutable, validated runtime tuning values for ball-alignment turn control. */
public final class BallAlignmentConfig {
	private static final double MIN_CORRECTION_DEG = 0.25;

	private final double turnKp;
	private final double turnKi;
	private final double turnKd;
	private final double derivativeFilter;
	private final double startCorrectionDeg;
	private final double stopCorrectionDeg;
	private final double maxTurnPower;
	private final double staticFrictionPower;
	private final double turnSlewRate;
	private final long commandHoldMs;

	private BallAlignmentConfig(double turnKp, double turnKi, double turnKd,
			double derivativeFilter, double startCorrectionDeg, double stopCorrectionDeg,
			double maxTurnPower, double staticFrictionPower, double turnSlewRate,
			long commandHoldMs) {
		this.turnKp = nonNegative(turnKp);
		this.turnKi = nonNegative(turnKi);
		this.turnKd = nonNegative(turnKd);
		this.derivativeFilter = nonNegative(derivativeFilter);
		this.startCorrectionDeg = Math.max(MIN_CORRECTION_DEG, finiteOr(startCorrectionDeg, MIN_CORRECTION_DEG));
		this.stopCorrectionDeg = clamp(finiteOr(stopCorrectionDeg, MIN_CORRECTION_DEG),
				MIN_CORRECTION_DEG, this.startCorrectionDeg);
		this.maxTurnPower = clamp(finiteOr(maxTurnPower, Double.MIN_VALUE), Double.MIN_VALUE, 1.0);
		this.staticFrictionPower = clamp(finiteOr(staticFrictionPower, 0.0), 0.0, this.maxTurnPower);
		this.turnSlewRate = nonNegative(turnSlewRate);
		this.commandHoldMs = Math.max(1L, commandHoldMs);
	}

	public static BallAlignmentConfig defaults() {
		return new BallAlignmentConfig(
				Constants.BALL_ALIGN_TURN_KP,
				Constants.BALL_ALIGN_TURN_KI,
				Constants.BALL_ALIGN_TURN_KD,
				Constants.BALL_ALIGN_TURN_DERIVATIVE_FILTER,
				Constants.BALL_ALIGN_START_CORRECTION_DEG,
				Constants.BALL_ALIGN_STOP_CORRECTION_DEG,
				Constants.BALL_ALIGN_MAX_TURN_POWER,
				Constants.BALL_ALIGN_STATIC_FRICTION_POWER,
				Constants.BALL_ALIGN_TURN_SLEW_RATE,
				Constants.BALL_ALIGN_COMMAND_HOLD_MS);
	}

	public double getTurnKp() { return turnKp; }
	public double getTurnKi() { return turnKi; }
	public double getTurnKd() { return turnKd; }
	public double getDerivativeFilter() { return derivativeFilter; }
	public double getStartCorrectionDeg() { return startCorrectionDeg; }
	public double getStopCorrectionDeg() { return stopCorrectionDeg; }
	public double getMaxTurnPower() { return maxTurnPower; }
	public double getStaticFrictionPower() { return staticFrictionPower; }
	public double getTurnSlewRate() { return turnSlewRate; }
	public long getCommandHoldMs() { return commandHoldMs; }

	public BallAlignmentConfig withTurnKp(double value) {
		return new BallAlignmentConfig(finiteOr(value, turnKp), turnKi, turnKd, derivativeFilter,
				startCorrectionDeg, stopCorrectionDeg, maxTurnPower, staticFrictionPower, turnSlewRate, commandHoldMs);
	}

	public BallAlignmentConfig withTurnKi(double value) {
		return new BallAlignmentConfig(turnKp, finiteOr(value, turnKi), turnKd, derivativeFilter,
				startCorrectionDeg, stopCorrectionDeg, maxTurnPower, staticFrictionPower, turnSlewRate, commandHoldMs);
	}

	public BallAlignmentConfig withTurnKd(double value) {
		return new BallAlignmentConfig(turnKp, turnKi, finiteOr(value, turnKd), derivativeFilter,
				startCorrectionDeg, stopCorrectionDeg, maxTurnPower, staticFrictionPower, turnSlewRate, commandHoldMs);
	}

	public BallAlignmentConfig withDerivativeFilter(double value) {
		return new BallAlignmentConfig(turnKp, turnKi, turnKd, finiteOr(value, derivativeFilter),
				startCorrectionDeg, stopCorrectionDeg, maxTurnPower, staticFrictionPower, turnSlewRate, commandHoldMs);
	}

	public BallAlignmentConfig withStartCorrectionDeg(double value) {
		return new BallAlignmentConfig(turnKp, turnKi, turnKd, derivativeFilter,
				finiteOr(value, startCorrectionDeg), stopCorrectionDeg, maxTurnPower, staticFrictionPower, turnSlewRate, commandHoldMs);
	}

	public BallAlignmentConfig withStopCorrectionDeg(double value) {
		return new BallAlignmentConfig(turnKp, turnKi, turnKd, derivativeFilter,
				startCorrectionDeg, finiteOr(value, stopCorrectionDeg), maxTurnPower, staticFrictionPower, turnSlewRate, commandHoldMs);
	}

	public BallAlignmentConfig withMaxTurnPower(double value) {
		return new BallAlignmentConfig(turnKp, turnKi, turnKd, derivativeFilter,
				startCorrectionDeg, stopCorrectionDeg, finiteOr(value, maxTurnPower), staticFrictionPower, turnSlewRate, commandHoldMs);
	}

	public BallAlignmentConfig withStaticFrictionPower(double value) {
		return new BallAlignmentConfig(turnKp, turnKi, turnKd, derivativeFilter,
				startCorrectionDeg, stopCorrectionDeg, maxTurnPower, finiteOr(value, staticFrictionPower), turnSlewRate, commandHoldMs);
	}

	public BallAlignmentConfig withTurnSlewRate(double value) {
		return new BallAlignmentConfig(turnKp, turnKi, turnKd, derivativeFilter,
				startCorrectionDeg, stopCorrectionDeg, maxTurnPower, staticFrictionPower, finiteOr(value, turnSlewRate), commandHoldMs);
	}

	public BallAlignmentConfig withCommandHoldMs(long value) {
		return new BallAlignmentConfig(turnKp, turnKi, turnKd, derivativeFilter,
				startCorrectionDeg, stopCorrectionDeg, maxTurnPower, staticFrictionPower, turnSlewRate, value);
	}

	private static double nonNegative(double value) {
		return Math.max(0.0, finiteOr(value, 0.0));
	}

	private static double finiteOr(double value, double fallback) {
		return Double.isFinite(value) ? value : fallback;
	}

	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
}
