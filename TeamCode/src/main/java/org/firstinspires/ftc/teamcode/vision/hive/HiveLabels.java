package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * Naming convention for HIVE CELLs and states (see docs/game/HIVE_NAMING.md).
 *
 * <p>Code and logs use FIELD-FIXED names: the FAR CELL is on the side of the field opposite
 * the audience (tile row 4, facing rows 5-6), the AUDIENCE CELL is on the audience side
 * (tile row 3, facing rows 1-2). These never change with robot position or alliance.</p>
 *
 * <p>Drivers get a DRIVER-RELATIVE hint. Red drivers stand at the left wall (seen from the
 * audience) facing right, so the audience side is on their RIGHT. Blue drivers stand at the
 * right wall facing left, so the audience side is on their LEFT (Competition Manual 9.5).</p>
 */
public final class HiveLabels {

	private HiveLabels() {}

	/** "LEFT" or "RIGHT": where the CELL is as seen by that alliance's drivers. */
	public static String driverSide(Alliance alliance, HiveCell cell) {
		boolean audienceOnRight = alliance == Alliance.RED;
		boolean isAudience = cell == HiveCell.AUDIENCE;
		return isAudience == audienceOnRight ? "RIGHT" : "LEFT";
	}

	/** Compact code for notes and logs: "R-F", "R-A", "B-F", "B-A". */
	public static String shortCode(Alliance alliance, HiveCell cell) {
		return (alliance == Alliance.RED ? "R" : "B") + "-" + (cell == HiveCell.FAR ? "F" : "A");
	}

	/** Field-fixed name of the HIVE state: "FAR UP", "AUDIENCE UP" or "UNKNOWN". */
	public static String stateName(HiveState state) {
		HiveCell up = state == null ? null : state.upCell();
		return up == null ? "UNKNOWN" : up + " UP";
	}

	/**
	 * Manual tile name (Figure 9-5) for a field position in the {@link HiveFieldModel} frame:
	 * columns A-F along x, rows 1-6 along y, 24 in per tile. "off field" outside 0-144.
	 */
	public static String tileName(double xIn, double yIn) {
		if (!(xIn >= 0.0 && xIn <= 144.0 && yIn >= 0.0 && yIn <= 144.0)) {
			return "off field";
		}
		int col = Math.min(5, (int) (xIn / 24.0));
		int row = Math.min(5, (int) (yIn / 24.0));
		return "" + (char) ('A' + col) + (row + 1);
	}

	/** Driver display, e.g. "FAR UP (your LEFT)". */
	public static String describe(Alliance alliance, HiveState state) {
		HiveCell up = state == null ? null : state.upCell();
		if (up == null) {
			return "UNKNOWN";
		}
		return up + " UP (your " + driverSide(alliance, up) + ")";
	}
}
