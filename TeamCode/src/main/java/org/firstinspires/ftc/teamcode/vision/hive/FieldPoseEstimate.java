package org.firstinspires.ftc.teamcode.vision.hive;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

/** Result of one {@link HiveLocalizer} run, in the field frame of {@link HiveFieldModel}. */
public final class FieldPoseEstimate {

	public enum Mode {
		/** Heading came from the gyro/odometry; x and y from the tags. */
		HEADING_GIVEN,
		/** x, y and heading all fitted from 2+ tags. */
		TAGS_ONLY,
		/** No estimate (see {@link #reason}). */
		NONE
	}

	public final boolean valid;
	public final Mode mode;
	public final double xIn;
	public final double yIn;
	public final double headingDeg;
	public final int tagsUsed;
	public final int tagsRejected;
	/** Short codes of the clusters used, with their position, e.g. "R-A RAISED". */
	public final List<String> clusters;
	/** RMS horizontal disagreement between tags after the fit, inches (fit quality). */
	public final double rmsResidualIn;
	/** RMS of measured minus modeled tag height, inches (checks camera mount and HIVE model). */
	public final double rmsHeightErrorIn;
	public final String reason;

	FieldPoseEstimate(boolean valid, Mode mode, double xIn, double yIn, double headingDeg,
			int tagsUsed, int tagsRejected, List<String> clusters, double rmsResidualIn,
			double rmsHeightErrorIn, String reason) {
		this.valid = valid;
		this.mode = mode;
		this.xIn = xIn;
		this.yIn = yIn;
		this.headingDeg = headingDeg;
		this.tagsUsed = tagsUsed;
		this.tagsRejected = tagsRejected;
		this.clusters = clusters == null ? Collections.<String>emptyList()
				: Collections.unmodifiableList(clusters);
		this.rmsResidualIn = rmsResidualIn;
		this.rmsHeightErrorIn = rmsHeightErrorIn;
		this.reason = reason;
	}

	static FieldPoseEstimate none(String reason, List<String> clusters, int rejected) {
		return new FieldPoseEstimate(false, Mode.NONE, Double.NaN, Double.NaN, Double.NaN,
				0, rejected, clusters, Double.NaN, Double.NaN, reason);
	}

	/** e.g. "x 34.2 y 18.7 h 71.5  (B1)" or "none: no HIVE tags". */
	public String summary() {
		if (!valid) {
			return "none: " + reason;
		}
		return String.format(Locale.US, "x %.1f y %.1f h %.1f  (%s)", xIn, yIn, headingDeg,
				HiveLabels.tileName(xIn, yIn));
	}
}
