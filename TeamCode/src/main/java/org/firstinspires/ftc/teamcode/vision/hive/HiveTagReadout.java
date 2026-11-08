package org.firstinspires.ftc.teamcode.vision.hive;

import java.util.Locale;

/**
 * Text formatting for the "Hive Tag Telemetry" OpMode. Pure Java so it can be unit-tested.
 * One row per visible HIVE AprilTag, labelled with its CELL short code (R-F, R-A, B-A, B-F).
 */
public final class HiveTagReadout {

	private HiveTagReadout() {}

	/** Label for a tag: "33 R-F", or "?? 7" for a non-HIVE tag. */
	public static String label(int id) {
		if (!HiveTagMap.isHiveTag(id)) {
			return "?? " + id;
		}
		return id + " " + HiveLabels.shortCode(HiveTagMap.allianceOf(id), HiveTagMap.cellOf(id));
	}

	/**
	 * Compact row: "tx -12.3 ty 8.1 ta 0.21 r 54.2 h 46.8". Range and height are in inches;
	 * NaN values (no 3D solve) print as "--".
	 */
	public static String row(double txDeg, double tyDeg, double areaPct, double rangeIn, double heightIn) {
		return String.format(Locale.US, "tx %s ty %s ta %s r %s h %s",
				num(txDeg, 1), num(tyDeg, 1), num(areaPct, 2), num(rangeIn, 1), num(heightIn, 1));
	}

	/** Detail row: camera-space position (x right, y down, z forward) in inches and tag yaw. */
	public static String detail(double xIn, double yIn, double zIn, double yawDeg) {
		return String.format(Locale.US, "cam x %s y %s z %s  yaw %s",
				num(xIn, 1), num(yIn, 1), num(zIn, 1), num(yawDeg, 0));
	}

	static String num(double v, int decimals) {
		if (Double.isNaN(v) || Double.isInfinite(v)) {
			return "--";
		}
		return String.format(Locale.US, "%." + decimals + "f", v);
	}
}
