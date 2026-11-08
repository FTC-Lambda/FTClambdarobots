package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * Field position of every HIVE AprilTag, for either position of its CELL.
 *
 * <p>FIELD FRAME (inches): origin at the field corner at tile A1 (audience side, red side),
 * +x toward column F, +y toward row 6 (the far wall), +z up from the tiles. Heading is
 * counter-clockwise from +x. Field center is (72, 72).</p>
 *
 * <p>Model (derived from Competition Manual Figures 9-9, 9-10, 9-15 to 9-17; see
 * docs/game/HIVE_LOCALIZATION.md). Each HIVE is a rigid see-saw that pivots about an axis
 * parallel to x at the field's y center. Its two CELLs sit on an arm a short distance below
 * the pivot. The 4-tag cluster is on each CELL's bottom face, facing straight down when the
 * HIVE is level. A TIP rotates the whole HIVE by +/- tilt, so a RAISED CELL's cluster moves up
 * and toward the field center, and both clusters then face down and toward the raised end.</p>
 *
 * <p>Defaults reproduce the manual's 30.6 in "bottom of HIVE" with a 30 deg tilt, and put the
 * cluster centers at about 49.1 in (raised) and 33.6 in (lowered). They are estimates from
 * drawings: measure the real HIVE and update {@link Params#defaults()}.</p>
 */
public final class HiveFieldModel {

	/** Geometry parameters, inches and degrees. */
	public static final class Params {
		public final double fieldCenterXIn;
		public final double fieldCenterYIn;
		/** Distance between the red and blue HIVE centers along x (manual: 25.5 in). */
		public final double hiveSpacingIn;
		/** Pivot axis height above the tiles (manual: 43.95 in). */
		public final double pivotHeightIn;
		/** Pivot to the center of a CELL's bottom face along the arm (18.84/2 + 12.04/2). */
		public final double armHalfLengthIn;
		/** How far the pivot sits above the arm / CELL bottom faces (fit to the 30.6 in low point). */
		public final double pivotAboveArmIn;
		/** HIVE rotation from level in either stable state. */
		public final double tiltDeg;
		/** Tag-center offsets within a cluster along x, in ascending-ID order (printable sheets). */
		public final double[] tagOffsetsIn;
		/** Tag centers' offset from the bottom-face center toward the CELL's outer end. */
		public final double faceOffsetIn;

		public Params(double fieldCenterXIn, double fieldCenterYIn, double hiveSpacingIn,
				double pivotHeightIn, double armHalfLengthIn, double pivotAboveArmIn, double tiltDeg,
				double[] tagOffsetsIn, double faceOffsetIn) {
			if (tagOffsetsIn == null || tagOffsetsIn.length != 4) {
				throw new IllegalArgumentException("tagOffsetsIn must have 4 entries");
			}
			this.fieldCenterXIn = fieldCenterXIn;
			this.fieldCenterYIn = fieldCenterYIn;
			this.hiveSpacingIn = hiveSpacingIn;
			this.pivotHeightIn = pivotHeightIn;
			this.armHalfLengthIn = armHalfLengthIn;
			this.pivotAboveArmIn = pivotAboveArmIn;
			this.tiltDeg = tiltDeg;
			this.tagOffsetsIn = tagOffsetsIn.clone();
			this.faceOffsetIn = faceOffsetIn;
		}

		public static Params defaults() {
			return new Params(72.0, 72.0, 25.5, 43.95, 15.44, 3.03, 30.0,
					new double[] {-6.5, -2.75, 2.75, 6.5}, 0.0);
		}
	}

	private final Params params;

	public HiveFieldModel(Params params) {
		this.params = params == null ? Params.defaults() : params;
	}

	public static HiveFieldModel defaults() {
		return new HiveFieldModel(Params.defaults());
	}

	public Params getParams() {
		return params;
	}

	/** x of a HIVE's center: red is on the red (low-x) side. */
	public double hiveCenterX(Alliance alliance) {
		double half = params.hiveSpacingIn / 2.0;
		return alliance == Alliance.RED ? params.fieldCenterXIn - half : params.fieldCenterXIn + half;
	}

	/** The position a CELL is in for a given HIVE state (UNKNOWN state gives null). */
	public static CellPosition positionFor(HiveCell cell, HiveState state) {
		HiveCell up = state == null ? null : state.upCell();
		if (up == null) {
			return null;
		}
		return up == cell ? CellPosition.RAISED : CellPosition.LOWERED;
	}

	/** Field position of the center of a CELL's tag cluster, or null if position is not RAISED/LOWERED. */
	public Vec3 clusterCenter(Alliance alliance, HiveCell cell, CellPosition position) {
		return point(alliance, cell, position, 0.0);
	}

	/** Field position of a tag's center, or null for a non-HIVE id or a position other than RAISED/LOWERED. */
	public Vec3 tagPosition(int id, CellPosition position) {
		if (!HiveTagMap.isHiveTag(id)) {
			return null;
		}
		Alliance alliance = HiveTagMap.allianceOf(id);
		HiveCell cell = HiveTagMap.cellOf(id);
		// Manual Figure 9-17 (top view, audience at the bottom): audience clusters read
		// 34 35 36 37 / 38 39 40 41 left to right (+x); far clusters read 33 32 31 30 /
		// 45 44 43 42, i.e. ascending IDs run toward -x.
		double along = params.tagOffsetsIn[(id - 30) % 4];
		double xDir = cell == HiveCell.AUDIENCE ? 1.0 : -1.0;
		return point(alliance, cell, position, xDir * along);
	}

	private Vec3 point(Alliance alliance, HiveCell cell, CellPosition position, double dx) {
		if (alliance == null || cell == null
				|| (position != CellPosition.RAISED && position != CellPosition.LOWERED)) {
			return null;
		}
		double s = cell == HiveCell.FAR ? 1.0 : -1.0;
		double alpha = Math.toRadians(params.tiltDeg) * (position == CellPosition.RAISED ? s : -s);
		// Point in the HIVE's level pose (y along the arm, z up from the pivot)...
		double yLevel = s * (params.armHalfLengthIn + params.faceOffsetIn);
		double zLevel = -params.pivotAboveArmIn;
		// ...rotated about the pivot axis (parallel to x); positive alpha raises the +y (far) end.
		double y = yLevel * Math.cos(alpha) - zLevel * Math.sin(alpha);
		double z = yLevel * Math.sin(alpha) + zLevel * Math.cos(alpha);
		return new Vec3(hiveCenterX(alliance) + dx, params.fieldCenterYIn + y, params.pivotHeightIn + z);
	}
}
