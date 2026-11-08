package org.firstinspires.ftc.teamcode.vision.hive;

/** One AprilTag as the Limelight saw it: 3D range plus crosshair-free angles. */
public final class TagSighting {
	public final int id;
	public final double rangeIn;
	public final double txDeg;
	public final double tyDeg;

	public TagSighting(int id, double rangeIn, double txDeg, double tyDeg) {
		this.id = id;
		this.rangeIn = rangeIn;
		this.txDeg = txDeg;
		this.tyDeg = tyDeg;
	}
}
