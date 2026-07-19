package org.firstinspires.ftc.teamcode.vision;

/**
 * DECODE ball class labels from the neural detector on pipeline 1.
 * Class names are matched case-insensitively against Limelight detector output.
 */
public enum BallColor {
	GREEN,
	PURPLE,
	UNKNOWN;

	public static BallColor fromClassName(String className) {
		if (className == null) {
			return UNKNOWN;
		}
		String n = className.trim().toLowerCase();
		if (n.contains("green")) {
			return GREEN;
		}
		if (n.contains("purple")) {
			return PURPLE;
		}
		return UNKNOWN;
	}
}
