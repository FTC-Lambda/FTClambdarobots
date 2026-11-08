package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;

/**
 * Maps BIOBUZZ HIVE AprilTag IDs (Competition Manual 9.9) to alliance and CELL.
 * Each CELL carries a cluster of 4 tags: RED far 30-33, RED audience 34-37,
 * BLUE audience 38-41, BLUE far 42-45.
 */
public final class HiveTagMap {
	private static final int FIRST_ID = 30;
	private static final int LAST_ID = 45;
	private static final int TAGS_PER_CELL = 4;

	private HiveTagMap() {}

	/** True if the id is one of the 16 HIVE tags (30-45). */
	public static boolean isHiveTag(int id) {
		return id >= FIRST_ID && id <= LAST_ID;
	}

	/** Alliance whose HIVE carries this tag, or null if it is not a HIVE tag. */
	public static Alliance allianceOf(int id) {
		if (!isHiveTag(id)) return null;
		return id < 38 ? Alliance.RED : Alliance.BLUE;
	}

	/** CELL (FAR or AUDIENCE) that carries this tag, or null if it is not a HIVE tag. */
	public static HiveCell cellOf(int id) {
		if (!isHiveTag(id)) return null;
		if (id < 34) return HiveCell.FAR;
		if (id < 38) return HiveCell.AUDIENCE;
		if (id < 42) return HiveCell.AUDIENCE;
		return HiveCell.FAR;
	}

	/** The 4 tag IDs on the given alliance's CELL, ascending. Returns a fresh array each call. */
	public static int[] tagIds(Alliance alliance, HiveCell cell) {
		int first;
		if (alliance == Alliance.RED) {
			first = cell == HiveCell.FAR ? 30 : 34;
		} else {
			first = cell == HiveCell.AUDIENCE ? 38 : 42;
		}
		int[] ids = new int[TAGS_PER_CELL];
		for (int i = 0; i < TAGS_PER_CELL; i++) {
			ids[i] = first + i;
		}
		return ids;
	}
}
