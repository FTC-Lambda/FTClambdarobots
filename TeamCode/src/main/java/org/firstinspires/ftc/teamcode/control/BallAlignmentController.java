package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.vision.BallTarget;

/**
 * FTC-independent turn controller for aligning the robot with a selected ball target.
 */
public final class BallAlignmentController {
	private static final double MIN_CONTROL_DT_SECONDS = 1.000001e-4;
	private static final double VALUE_EPSILON = 1e-12;

	public enum Action {
		NO_TARGET,
		CENTERED,
		CORRECTING,
		HOLDING_COMMAND,
		COMMAND_EXPIRED,
		REVERSAL_GUARD
	}

	/** Immutable diagnostics and turn command from one controller update. */
	public static final class Result {
		private final double bearing;
		private final double p;
		private final double i;
		private final double d;
		private final double rawPid;
		private final double requestedTurn;
		private final double appliedTurn;
		private final boolean correctionActive;
		private final boolean staticFrictionApplied;
		private final boolean slewLimited;
		private final Action action;

		private Result(double bearing, double p, double i, double d, double rawPid,
				double requestedTurn, double appliedTurn, boolean correctionActive,
				boolean staticFrictionApplied, boolean slewLimited, Action action) {
			this.bearing = bearing;
			this.p = p;
			this.i = i;
			this.d = d;
			this.rawPid = rawPid;
			this.requestedTurn = requestedTurn;
			this.appliedTurn = appliedTurn;
			this.correctionActive = correctionActive;
			this.staticFrictionApplied = staticFrictionApplied;
			this.slewLimited = slewLimited;
			this.action = action;
		}

		public double getBearing() { return bearing; }
		public double getBearingDeg() { return bearing; }
		public double getP() { return p; }
		public double getI() { return i; }
		public double getD() { return d; }
		public double getRawPid() { return rawPid; }
		public double getRequestedTurn() { return requestedTurn; }
		public double getAppliedTurn() { return appliedTurn; }
		public boolean isCorrectionActive() { return correctionActive; }
		public boolean isStaticFrictionApplied() { return staticFrictionApplied; }
		public boolean isSlewLimited() { return slewLimited; }
		public Action getAction() { return action; }
	}

	private BallAlignmentConfig config;
	private PIDController pid;
	private SlewRateLimiter slewLimiter;

	private long lastFreshMs = Long.MIN_VALUE;
	private long lastUpdateMs = Long.MIN_VALUE;
	private double requestedTurn;
	private double appliedTurn;
	private boolean correctionActive;
	private int pendingReverseSign;

	private double lastP;
	private double lastI;
	private double lastD;
	private double lastRawPid;
	private boolean staticFrictionApplied;

	public BallAlignmentController(BallAlignmentConfig config) {
		setConfig(config);
	}

	public void setConfig(BallAlignmentConfig config) {
		if (config == null) {
			throw new IllegalArgumentException("config must not be null");
		}
		this.config = config;
		pid = new PIDController(
				config.getTurnKp(),
				config.getTurnKi(),
				config.getTurnKd(),
				Double.MAX_VALUE,
				Double.MAX_VALUE,
				config.getDerivativeFilter());
		slewLimiter = new SlewRateLimiter(config.getTurnSlewRate());
		reset();
	}

	public Result update(BallTarget target, long nowMs, double loopDtSeconds) {
		double bearing = validBearing(target);
		if (!validTiming(nowMs, loopDtSeconds)) {
			return stopAndResetControl(Action.NO_TARGET, bearing);
		}
		lastUpdateMs = nowMs;

		if (target == null || !target.isValid() || !Double.isFinite(target.getHorizontalErrorDeg())
				|| target.getAgeMs() < 0) {
			return stopAndResetControl(Action.NO_TARGET, bearing);
		}

		if (target.isFresh()) {
			return updateFresh(bearing, nowMs, loopDtSeconds);
		}
		return updateHeld(bearing, nowMs, loopDtSeconds, target.getAgeMs());
	}

	public void reset() {
		lastUpdateMs = Long.MIN_VALUE;
		clearControlState();
	}

	private Result updateFresh(double bearing, long nowMs, double loopDtSeconds) {
		double absBearing = Math.abs(bearing);
		if (!correctionActive && absBearing > config.getStartCorrectionDeg()) {
			correctionActive = true;
		}
		if (!correctionActive || absBearing <= config.getStopCorrectionDeg()) {
			return stopAndResetControl(Action.CENTERED, bearing);
		}

		double measurementDt = lastFreshMs == Long.MIN_VALUE
				? safeLoopDt(loopDtSeconds)
				: Math.max(MIN_CONTROL_DT_SECONDS, (nowMs - lastFreshMs) / 1000.0);
		double rawPid = pid.calculate(bearing, 0.0, measurementDt);
		if (!Double.isFinite(rawPid)) {
			return stopAndResetControl(Action.NO_TARGET, bearing);
		}

		lastFreshMs = nowMs;
		lastP = pid.getLastP();
		lastI = pid.getLastI();
		lastD = pid.getLastD();
		lastRawPid = rawPid;

		double request = clamp(rawPid, -config.getMaxTurnPower(), config.getMaxTurnPower());
		staticFrictionApplied = request != 0.0
				&& Math.abs(request) < config.getStaticFrictionPower();
		if (staticFrictionApplied) {
			request = Math.copySign(config.getStaticFrictionPower(), request);
		}
		requestedTurn = request;

		int requestSign = sign(request);
		if (pendingReverseSign != 0) {
			pendingReverseSign = 0;
		} else if (requestSign != 0 && sign(appliedTurn) != 0
				&& requestSign != sign(appliedTurn)) {
			pendingReverseSign = requestSign;
			appliedTurn = 0.0;
			seedLimiterAtZero();
			return result(bearing, false, Action.REVERSAL_GUARD);
		}

		return applyRequested(bearing, request, loopDtSeconds, Action.CORRECTING);
	}

	private Result updateHeld(double bearing, long nowMs, double loopDtSeconds, long targetAgeMs) {
		boolean hasHeldCommand = lastFreshMs != Long.MIN_VALUE
				&& targetAgeMs <= config.getCommandHoldMs()
				&& nowMs - lastFreshMs <= config.getCommandHoldMs();
		if (hasHeldCommand) {
			if (pendingReverseSign != 0) {
				appliedTurn = 0.0;
				return result(bearing, false, Action.REVERSAL_GUARD);
			}
			return applyRequested(
					bearing, requestedTurn, loopDtSeconds, Action.HOLDING_COMMAND);
		}

		requestedTurn = 0.0;
		staticFrictionApplied = false;
		pendingReverseSign = 0;
		return applyRequested(bearing, 0.0, loopDtSeconds, Action.COMMAND_EXPIRED);
	}

	private Result applyRequested(double bearing, double request, double loopDtSeconds,
			Action action) {
		appliedTurn = clamp(
				slewLimiter.calculate(request, safeLoopDt(loopDtSeconds)),
				-config.getMaxTurnPower(),
				config.getMaxTurnPower());
		boolean slewLimited = Math.abs(appliedTurn - request) > VALUE_EPSILON;
		return result(bearing, slewLimited, action);
	}

	private Result stopAndResetControl(Action action, double bearing) {
		clearControlState();
		return result(bearing, false, action);
	}

	private void clearControlState() {
		lastFreshMs = Long.MIN_VALUE;
		requestedTurn = 0.0;
		appliedTurn = 0.0;
		correctionActive = false;
		pendingReverseSign = 0;
		lastP = 0.0;
		lastI = 0.0;
		lastD = 0.0;
		lastRawPid = 0.0;
		staticFrictionApplied = false;
		pid.reset();
		seedLimiterAtZero();
	}

	private void seedLimiterAtZero() {
		slewLimiter.reset();
		slewLimiter.calculate(0.0, 0.0);
	}

	private Result result(double bearing, boolean slewLimited, Action action) {
		return new Result(
				bearing,
				lastP,
				lastI,
				lastD,
				lastRawPid,
				requestedTurn,
				appliedTurn,
				correctionActive,
				staticFrictionApplied,
				slewLimited,
				action);
	}

	private boolean validTiming(long nowMs, double loopDtSeconds) {
		return nowMs >= 0
				&& Double.isFinite(loopDtSeconds)
				&& loopDtSeconds > 0.0
				&& (lastUpdateMs == Long.MIN_VALUE || nowMs > lastUpdateMs);
	}

	private static double validBearing(BallTarget target) {
		if (target == null || !Double.isFinite(target.getHorizontalErrorDeg())) {
			return 0.0;
		}
		return target.getHorizontalErrorDeg();
	}

	private static double safeLoopDt(double loopDtSeconds) {
		return Math.max(MIN_CONTROL_DT_SECONDS, loopDtSeconds);
	}

	private static int sign(double value) {
		return value > 0.0 ? 1 : value < 0.0 ? -1 : 0;
	}

	private static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}
}
