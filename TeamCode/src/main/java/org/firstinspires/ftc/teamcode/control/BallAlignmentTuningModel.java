package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.vision.BallTargetingConfig;

/**
 * Pure-Java state model for selecting and adjusting ball-alignment parameters on the driver
 * station. Configuration changes are applied through their immutable withers.
 */
public final class BallAlignmentTuningModel {
	public enum ChangeDomain {
		ALIGNMENT,
		TARGETING
	}

	public enum Parameter {
		TURN_KP("turn kP", 0.002, 0.010, ChangeDomain.ALIGNMENT),
		TURN_KI("turn kI", 0.0005, 0.0020, ChangeDomain.ALIGNMENT),
		TURN_KD("turn kD", 0.001, 0.005, ChangeDomain.ALIGNMENT),
		DERIVATIVE_FILTER("derivative filter", 0.05, 0.20, ChangeDomain.ALIGNMENT),
		START_DEADBAND("start deadband", 0.25, 1.00, ChangeDomain.ALIGNMENT),
		STOP_DEADBAND("stop deadband", 0.25, 1.00, ChangeDomain.ALIGNMENT),
		MAX_TURN_POWER("max turn power", 0.05, 0.10, ChangeDomain.ALIGNMENT),
		STATIC_FRICTION("static friction", 0.01, 0.05, ChangeDomain.ALIGNMENT),
		TURN_SLEW_RATE("turn slew rate", 0.5, 2.0, ChangeDomain.ALIGNMENT),
		COMMAND_HOLD_MS("command hold ms", 25, 100, ChangeDomain.ALIGNMENT),
		SWITCH_CONFIRM_FRAMES("switch confirm frames", 1, 1, ChangeDomain.TARGETING),
		AIM_FILTER_GAIN("aim filter gain", 0.05, 0.20, ChangeDomain.TARGETING);

		private final String displayName;
		private final double fineIncrement;
		private final double coarseIncrement;
		private final ChangeDomain domain;

		Parameter(String displayName, double fineIncrement, double coarseIncrement,
				ChangeDomain domain) {
			this.displayName = displayName;
			this.fineIncrement = fineIncrement;
			this.coarseIncrement = coarseIncrement;
			this.domain = domain;
		}

		public String getDisplayName() { return displayName; }
		public double getFineIncrement() { return fineIncrement; }
		public double getCoarseIncrement() { return coarseIncrement; }
		public ChangeDomain getDomain() { return domain; }
	}

	private int selectedIndex;
	private BallAlignmentConfig alignmentConfig;
	private BallTargetingConfig targetingConfig;

	public BallAlignmentTuningModel() {
		this(BallAlignmentConfig.defaults(), BallTargetingConfig.defaults());
	}

	public BallAlignmentTuningModel(BallAlignmentConfig alignmentConfig,
			BallTargetingConfig targetingConfig) {
		if (alignmentConfig == null || targetingConfig == null) {
			throw new IllegalArgumentException("Configurations must not be null");
		}
		this.alignmentConfig = alignmentConfig;
		this.targetingConfig = targetingConfig;
	}

	public void selectPrevious() {
		selectedIndex = (selectedIndex + Parameter.values().length - 1) % Parameter.values().length;
	}

	public void selectNext() {
		selectedIndex = (selectedIndex + 1) % Parameter.values().length;
	}

	/** Adjusts the selected parameter and returns the configuration domain it belongs to. */
	public ChangeDomain adjust(int direction, boolean coarse) {
		Parameter parameter = getSelectedParameter();
		int normalizedDirection = Integer.compare(direction, 0);
		if (normalizedDirection == 0) {
			return parameter.getDomain();
		}
		double increment = coarse ? parameter.getCoarseIncrement() : parameter.getFineIncrement();
		double delta = normalizedDirection * increment;

		switch (parameter) {
			case TURN_KP:
				alignmentConfig = alignmentConfig.withTurnKp(alignmentConfig.getTurnKp() + delta);
				break;
			case TURN_KI:
				alignmentConfig = alignmentConfig.withTurnKi(alignmentConfig.getTurnKi() + delta);
				break;
			case TURN_KD:
				alignmentConfig = alignmentConfig.withTurnKd(alignmentConfig.getTurnKd() + delta);
				break;
			case DERIVATIVE_FILTER:
				alignmentConfig = alignmentConfig.withDerivativeFilter(alignmentConfig.getDerivativeFilter() + delta);
				break;
			case START_DEADBAND:
				alignmentConfig = alignmentConfig.withStartCorrectionDeg(Math.max(
						alignmentConfig.getStopCorrectionDeg(),
						alignmentConfig.getStartCorrectionDeg() + delta));
				break;
			case STOP_DEADBAND:
				alignmentConfig = alignmentConfig.withStopCorrectionDeg(Math.min(
						alignmentConfig.getStartCorrectionDeg(),
						alignmentConfig.getStopCorrectionDeg() + delta));
				break;
			case MAX_TURN_POWER:
				alignmentConfig = alignmentConfig.withMaxTurnPower(Math.max(
						alignmentConfig.getStaticFrictionPower(),
						alignmentConfig.getMaxTurnPower() + delta));
				break;
			case STATIC_FRICTION:
				alignmentConfig = alignmentConfig.withStaticFrictionPower(Math.min(
						alignmentConfig.getMaxTurnPower(),
						alignmentConfig.getStaticFrictionPower() + delta));
				break;
			case TURN_SLEW_RATE:
				alignmentConfig = alignmentConfig.withTurnSlewRate(alignmentConfig.getTurnSlewRate() + delta);
				break;
			case COMMAND_HOLD_MS:
				alignmentConfig = alignmentConfig.withCommandHoldMs(
						alignmentConfig.getCommandHoldMs() + (long) delta);
				break;
			case SWITCH_CONFIRM_FRAMES:
				targetingConfig = targetingConfig.withSwitchConfirmFrames(
						targetingConfig.getSwitchConfirmFrames() + (int) delta);
				break;
			case AIM_FILTER_GAIN:
				targetingConfig = targetingConfig.withAimFilterGain(targetingConfig.getAimFilterGain() + delta);
				break;
			default:
				throw new IllegalStateException("Unknown parameter: " + parameter);
		}
		return parameter.getDomain();
	}

	public Parameter getSelectedParameter() {
		return Parameter.values()[selectedIndex];
	}

	public BallAlignmentConfig getAlignmentConfig() {
		return alignmentConfig;
	}

	public BallTargetingConfig getTargetingConfig() {
		return targetingConfig;
	}

	public double getSelectedValue() {
		switch (getSelectedParameter()) {
			case TURN_KP: return alignmentConfig.getTurnKp();
			case TURN_KI: return alignmentConfig.getTurnKi();
			case TURN_KD: return alignmentConfig.getTurnKd();
			case DERIVATIVE_FILTER: return alignmentConfig.getDerivativeFilter();
			case START_DEADBAND: return alignmentConfig.getStartCorrectionDeg();
			case STOP_DEADBAND: return alignmentConfig.getStopCorrectionDeg();
			case MAX_TURN_POWER: return alignmentConfig.getMaxTurnPower();
			case STATIC_FRICTION: return alignmentConfig.getStaticFrictionPower();
			case TURN_SLEW_RATE: return alignmentConfig.getTurnSlewRate();
			case COMMAND_HOLD_MS: return alignmentConfig.getCommandHoldMs();
			case SWITCH_CONFIRM_FRAMES: return targetingConfig.getSwitchConfirmFrames();
			case AIM_FILTER_GAIN: return targetingConfig.getAimFilterGain();
			default: throw new IllegalStateException("Unknown selected parameter");
		}
	}

	public double getSelectedDefaultValue() {
		BallAlignmentConfig defaultAlignment = BallAlignmentConfig.defaults();
		BallTargetingConfig defaultTargeting = BallTargetingConfig.defaults();
		switch (getSelectedParameter()) {
			case TURN_KP: return defaultAlignment.getTurnKp();
			case TURN_KI: return defaultAlignment.getTurnKi();
			case TURN_KD: return defaultAlignment.getTurnKd();
			case DERIVATIVE_FILTER: return defaultAlignment.getDerivativeFilter();
			case START_DEADBAND: return defaultAlignment.getStartCorrectionDeg();
			case STOP_DEADBAND: return defaultAlignment.getStopCorrectionDeg();
			case MAX_TURN_POWER: return defaultAlignment.getMaxTurnPower();
			case STATIC_FRICTION: return defaultAlignment.getStaticFrictionPower();
			case TURN_SLEW_RATE: return defaultAlignment.getTurnSlewRate();
			case COMMAND_HOLD_MS: return defaultAlignment.getCommandHoldMs();
			case SWITCH_CONFIRM_FRAMES: return defaultTargeting.getSwitchConfirmFrames();
			case AIM_FILTER_GAIN: return defaultTargeting.getAimFilterGain();
			default: throw new IllegalStateException("Unknown selected parameter");
		}
	}

	public double getFineIncrement() {
		return getSelectedParameter().getFineIncrement();
	}

	public double getCoarseIncrement() {
		return getSelectedParameter().getCoarseIncrement();
	}
}
