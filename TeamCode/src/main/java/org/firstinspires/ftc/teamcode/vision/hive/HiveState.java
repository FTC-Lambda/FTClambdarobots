package org.firstinspires.ftc.teamcode.vision.hive;

/** Which CELL of a HIVE currently faces up (or UNKNOWN before the first confirmation). */
public enum HiveState {
	FAR_CELL_UP,
	AUDIENCE_CELL_UP,
	UNKNOWN;

	/** The CELL facing up, or null for UNKNOWN. */
	public HiveCell upCell() {
		switch (this) {
			case FAR_CELL_UP:
				return HiveCell.FAR;
			case AUDIENCE_CELL_UP:
				return HiveCell.AUDIENCE;
			default:
				return null;
		}
	}

	/** The state in which {@code cell} faces up; UNKNOWN for null. */
	public static HiveState withUp(HiveCell cell) {
		if (cell == HiveCell.FAR) {
			return FAR_CELL_UP;
		}
		if (cell == HiveCell.AUDIENCE) {
			return AUDIENCE_CELL_UP;
		}
		return UNKNOWN;
	}
}
