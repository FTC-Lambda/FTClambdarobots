package org.firstinspires.ftc.teamcode.vision.hive;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class HiveTagReadoutTest {

	@Test
	public void labelsHiveTagsWithCellCode() {
		assertEquals("30 R-F", HiveTagReadout.label(30));
		assertEquals("37 R-A", HiveTagReadout.label(37));
		assertEquals("38 B-A", HiveTagReadout.label(38));
		assertEquals("45 B-F", HiveTagReadout.label(45));
		assertEquals("?? 20", HiveTagReadout.label(20));
	}

	@Test
	public void rowPrintsDashesForMissing3d() {
		assertEquals("tx -12.3 ty 8.1 ta 0.21 r 54.2 h 46.8",
				HiveTagReadout.row(-12.34, 8.06, 0.2149, 54.21, 46.84));
		assertEquals("tx 1.0 ty 2.0 ta 0.50 r -- h --",
				HiveTagReadout.row(1.0, 2.0, 0.5, Double.NaN, Double.NaN));
	}

	@Test
	public void detailRow() {
		assertEquals("cam x 1.5 y -20.0 z 60.3  yaw 12",
				HiveTagReadout.detail(1.5, -20.0, 60.25, 12.4));
	}
}
