package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class HiveLabelsTest {

	@Test
	public void redDriversSeeAudienceOnTheirRight() {
		assertEquals("RIGHT", HiveLabels.driverSide(Alliance.RED, HiveCell.AUDIENCE));
		assertEquals("LEFT", HiveLabels.driverSide(Alliance.RED, HiveCell.FAR));
	}

	@Test
	public void blueDriversSeeAudienceOnTheirLeft() {
		assertEquals("LEFT", HiveLabels.driverSide(Alliance.BLUE, HiveCell.AUDIENCE));
		assertEquals("RIGHT", HiveLabels.driverSide(Alliance.BLUE, HiveCell.FAR));
	}

	@Test
	public void shortCodes() {
		assertEquals("R-F", HiveLabels.shortCode(Alliance.RED, HiveCell.FAR));
		assertEquals("R-A", HiveLabels.shortCode(Alliance.RED, HiveCell.AUDIENCE));
		assertEquals("B-F", HiveLabels.shortCode(Alliance.BLUE, HiveCell.FAR));
		assertEquals("B-A", HiveLabels.shortCode(Alliance.BLUE, HiveCell.AUDIENCE));
	}

	@Test
	public void stateNamesAndDriverDescriptions() {
		assertEquals("FAR UP", HiveLabels.stateName(HiveState.FAR_CELL_UP));
		assertEquals("AUDIENCE UP", HiveLabels.stateName(HiveState.AUDIENCE_CELL_UP));
		assertEquals("UNKNOWN", HiveLabels.stateName(HiveState.UNKNOWN));
		assertEquals("UNKNOWN", HiveLabels.stateName(null));
		assertEquals("FAR UP (your LEFT)", HiveLabels.describe(Alliance.RED, HiveState.FAR_CELL_UP));
		assertEquals("AUDIENCE UP (your LEFT)",
				HiveLabels.describe(Alliance.BLUE, HiveState.AUDIENCE_CELL_UP));
		assertEquals("UNKNOWN", HiveLabels.describe(Alliance.BLUE, HiveState.UNKNOWN));
	}
}
