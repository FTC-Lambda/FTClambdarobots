package org.firstinspires.ftc.teamcode.vision;

/** Immutable, validated runtime tuning values for ball target selection and aiming. */
public final class BallTargetingConfig {
	private static final int MIN_SWITCH_CONFIRM_FRAMES = 1;
	private static final int MAX_SWITCH_CONFIRM_FRAMES = 10;

	private final int switchConfirmFrames;
	private final double aimFilterGain;

	private BallTargetingConfig(int switchConfirmFrames, double aimFilterGain) {
		this.switchConfirmFrames = Math.max(MIN_SWITCH_CONFIRM_FRAMES,
				Math.min(MAX_SWITCH_CONFIRM_FRAMES, switchConfirmFrames));
		this.aimFilterGain = Math.max(Double.MIN_VALUE,
				Math.min(1.0, finiteOr(aimFilterGain, Double.MIN_VALUE)));
	}

	public static BallTargetingConfig defaults() {
		return new BallTargetingConfig(
				BallVisionConfig.BALL_TARGET_SWITCH_CONFIRM_FRAMES,
				BallVisionConfig.BALL_TARGET_AIM_FILTER_GAIN);
	}

	public int getSwitchConfirmFrames() { return switchConfirmFrames; }
	public double getAimFilterGain() { return aimFilterGain; }

	public BallTargetingConfig withSwitchConfirmFrames(int value) {
		return new BallTargetingConfig(value, aimFilterGain);
	}

	public BallTargetingConfig withAimFilterGain(double value) {
		return new BallTargetingConfig(switchConfirmFrames, finiteOr(value, aimFilterGain));
	}

	private static double finiteOr(double value, double fallback) {
		return Double.isFinite(value) ? value : fallback;
	}
}
