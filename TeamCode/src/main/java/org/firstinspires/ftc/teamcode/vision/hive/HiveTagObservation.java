package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;

/** Immutable: one AprilTag from a HIVE CELL with its estimated height above the floor. */
public final class HiveTagObservation {
	private final int tagId;
	private final Alliance alliance;
	private final HiveCell cell;
	private final double heightIn;

	public HiveTagObservation(int tagId, Alliance alliance, HiveCell cell, double heightIn) {
		this.tagId = tagId;
		this.alliance = alliance;
		this.cell = cell;
		this.heightIn = heightIn;
	}

	public int getTagId() { return tagId; }
	public Alliance getAlliance() { return alliance; }
	public HiveCell getCell() { return cell; }
	public double getHeightIn() { return heightIn; }
}
