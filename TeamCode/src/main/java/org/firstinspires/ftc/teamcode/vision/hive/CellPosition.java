package org.firstinspires.ftc.teamcode.vision.hive;

/** Classification of one CELL's tag-cluster height against the raised threshold. */
public enum CellPosition {
	RAISED,
	LOWERED,
	/** Median height lies inside the hysteresis band around the threshold. */
	AMBIGUOUS,
	/** Too few plausible tags were visible for this CELL. */
	NOT_SEEN
}
