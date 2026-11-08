package org.firstinspires.ftc.teamcode.vision.hive;

/**
 * The two CELLs of a HIVE. FAR is on the side of the field opposite the audience, AUDIENCE is on
 * the audience side. Exactly one of them faces up at a time.
 */
public enum HiveCell {
	FAR,
	AUDIENCE;

	/** The opposite CELL of the same HIVE (FAR.other() == AUDIENCE). */
	public HiveCell other() {
		return this == FAR ? AUDIENCE : FAR;
	}
}
