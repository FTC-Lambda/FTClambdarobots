package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.junit.Test;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class HiveTagMapTest {

	private static void checkRange(int first, Alliance alliance, HiveCell cell) {
		for (int id = first; id < first + 4; id++) {
			assertTrue("isHiveTag " + id, HiveTagMap.isHiveTag(id));
			assertEquals("alliance of " + id, alliance, HiveTagMap.allianceOf(id));
			assertEquals("cell of " + id, cell, HiveTagMap.cellOf(id));
		}
	}

	@Test
	public void mapsEveryHiveTag() {
		checkRange(30, Alliance.RED, HiveCell.FAR);
		checkRange(34, Alliance.RED, HiveCell.AUDIENCE);
		checkRange(38, Alliance.BLUE, HiveCell.AUDIENCE);
		checkRange(42, Alliance.BLUE, HiveCell.FAR);
	}

	@Test
	public void nonHiveTagsAreRejected() {
		int[] others = {29, 46, 20, 24, -1};
		for (int id : others) {
			assertFalse("isHiveTag " + id, HiveTagMap.isHiveTag(id));
			assertNull("alliance of " + id, HiveTagMap.allianceOf(id));
			assertNull("cell of " + id, HiveTagMap.cellOf(id));
		}
	}

	@Test
	public void tagIdsAreCorrectAndAscending() {
		assertArrayEquals(new int[]{30, 31, 32, 33}, HiveTagMap.tagIds(Alliance.RED, HiveCell.FAR));
		assertArrayEquals(new int[]{34, 35, 36, 37}, HiveTagMap.tagIds(Alliance.RED, HiveCell.AUDIENCE));
		assertArrayEquals(new int[]{38, 39, 40, 41}, HiveTagMap.tagIds(Alliance.BLUE, HiveCell.AUDIENCE));
		assertArrayEquals(new int[]{42, 43, 44, 45}, HiveTagMap.tagIds(Alliance.BLUE, HiveCell.FAR));
	}

	@Test
	public void tagIdsAgreeWithLookups() {
		for (Alliance a : Alliance.values()) {
			for (HiveCell c : HiveCell.values()) {
				for (int id : HiveTagMap.tagIds(a, c)) {
					assertEquals(a, HiveTagMap.allianceOf(id));
					assertEquals(c, HiveTagMap.cellOf(id));
				}
			}
		}
	}

	@Test
	public void tagIdsReturnsFreshCopy() {
		int[] first = HiveTagMap.tagIds(Alliance.RED, HiveCell.FAR);
		first[0] = 99;
		first[3] = -5;
		assertArrayEquals(new int[]{30, 31, 32, 33}, HiveTagMap.tagIds(Alliance.RED, HiveCell.FAR));
	}

	@Test
	public void otherCellFlips() {
		assertEquals(HiveCell.AUDIENCE, HiveCell.FAR.other());
		assertEquals(HiveCell.FAR, HiveCell.AUDIENCE.other());
	}
}
