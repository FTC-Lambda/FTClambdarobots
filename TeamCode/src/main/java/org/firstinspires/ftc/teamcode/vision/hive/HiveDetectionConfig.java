package org.firstinspires.ftc.teamcode.vision.hive;

/** Immutable, validated tuning values for HIVE state detection. */
public final class HiveDetectionConfig {
	public final double raisedThresholdIn;
	public final double hysteresisIn;
	public final double minPlausibleHeightIn;
	public final double maxPlausibleHeightIn;
	public final int minTagsPerCell;
	public final int confirmFrames;
	public final long staleAfterMs;

	/**
	 * @throws IllegalArgumentException if any double is non-finite, hysteresis or staleAfterMs is
	 *     negative, confirmFrames or minTagsPerCell is below 1, or min plausible >= max plausible
	 */
	public HiveDetectionConfig(double raisedThresholdIn, double hysteresisIn,
			double minPlausibleHeightIn, double maxPlausibleHeightIn,
			int minTagsPerCell, int confirmFrames, long staleAfterMs) {
		requireFinite(raisedThresholdIn, "raisedThresholdIn");
		requireFinite(hysteresisIn, "hysteresisIn");
		requireFinite(minPlausibleHeightIn, "minPlausibleHeightIn");
		requireFinite(maxPlausibleHeightIn, "maxPlausibleHeightIn");
		if (hysteresisIn < 0.0) {
			throw new IllegalArgumentException("hysteresisIn must be >= 0");
		}
		if (minPlausibleHeightIn >= maxPlausibleHeightIn) {
			throw new IllegalArgumentException(
					"minPlausibleHeightIn must be < maxPlausibleHeightIn");
		}
		if (minTagsPerCell < 1) {
			throw new IllegalArgumentException("minTagsPerCell must be >= 1");
		}
		if (confirmFrames < 1) {
			throw new IllegalArgumentException("confirmFrames must be >= 1");
		}
		if (staleAfterMs < 0L) {
			throw new IllegalArgumentException("staleAfterMs must be >= 0");
		}
		this.raisedThresholdIn = raisedThresholdIn;
		this.hysteresisIn = hysteresisIn;
		this.minPlausibleHeightIn = minPlausibleHeightIn;
		this.maxPlausibleHeightIn = maxPlausibleHeightIn;
		this.minTagsPerCell = minTagsPerCell;
		this.confirmFrames = confirmFrames;
		this.staleAfterMs = staleAfterMs;
	}

	public static HiveDetectionConfig defaults() {
		return new HiveDetectionConfig(40.0, 2.0, 15.0, 80.0, 1, 3, 1500L);
	}

	public HiveDetectionConfig withRaisedThresholdIn(double v) {
		return new HiveDetectionConfig(v, hysteresisIn, minPlausibleHeightIn,
				maxPlausibleHeightIn, minTagsPerCell, confirmFrames, staleAfterMs);
	}

	public HiveDetectionConfig withHysteresisIn(double v) {
		return new HiveDetectionConfig(raisedThresholdIn, v, minPlausibleHeightIn,
				maxPlausibleHeightIn, minTagsPerCell, confirmFrames, staleAfterMs);
	}

	private static void requireFinite(double value, String name) {
		if (Double.isNaN(value) || Double.isInfinite(value)) {
			throw new IllegalArgumentException(name + " must be finite");
		}
	}
}
