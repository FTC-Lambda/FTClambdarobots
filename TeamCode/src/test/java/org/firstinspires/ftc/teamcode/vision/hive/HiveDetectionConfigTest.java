package org.firstinspires.ftc.teamcode.vision.hive;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.fail;

public class HiveDetectionConfigTest {

	private static final double EPS = 1e-9;

	@Test
	public void defaultsMatchContract() {
		HiveDetectionConfig c = HiveDetectionConfig.defaults();
		assertEquals(40.0, c.raisedThresholdIn, EPS);
		assertEquals(2.0, c.hysteresisIn, EPS);
		assertEquals(15.0, c.minPlausibleHeightIn, EPS);
		assertEquals(80.0, c.maxPlausibleHeightIn, EPS);
		assertEquals(1, c.minTagsPerCell);
		assertEquals(3, c.confirmFrames);
		assertEquals(1500L, c.staleAfterMs);
	}

	@Test
	public void constructorArgumentOrderMatchesFields() {
		HiveDetectionConfig c = new HiveDetectionConfig(41.0, 1.5, 10.0, 90.0, 2, 4, 900L);
		assertEquals(41.0, c.raisedThresholdIn, EPS);
		assertEquals(1.5, c.hysteresisIn, EPS);
		assertEquals(10.0, c.minPlausibleHeightIn, EPS);
		assertEquals(90.0, c.maxPlausibleHeightIn, EPS);
		assertEquals(2, c.minTagsPerCell);
		assertEquals(4, c.confirmFrames);
		assertEquals(900L, c.staleAfterMs);
	}

	@Test
	public void withersReturnCopiesAndKeepOtherFields() {
		HiveDetectionConfig base = new HiveDetectionConfig(41.0, 1.5, 10.0, 90.0, 2, 4, 900L);

		HiveDetectionConfig t = base.withRaisedThresholdIn(44.0);
		assertNotSame(base, t);
		assertEquals(44.0, t.raisedThresholdIn, EPS);
		assertEquals(41.0, base.raisedThresholdIn, EPS);
		assertEquals(1.5, t.hysteresisIn, EPS);
		assertEquals(10.0, t.minPlausibleHeightIn, EPS);
		assertEquals(90.0, t.maxPlausibleHeightIn, EPS);
		assertEquals(2, t.minTagsPerCell);
		assertEquals(4, t.confirmFrames);
		assertEquals(900L, t.staleAfterMs);

		HiveDetectionConfig h = base.withHysteresisIn(3.0);
		assertNotSame(base, h);
		assertEquals(3.0, h.hysteresisIn, EPS);
		assertEquals(1.5, base.hysteresisIn, EPS);
		assertEquals(41.0, h.raisedThresholdIn, EPS);
		assertEquals(10.0, h.minPlausibleHeightIn, EPS);
		assertEquals(90.0, h.maxPlausibleHeightIn, EPS);
		assertEquals(2, h.minTagsPerCell);
		assertEquals(4, h.confirmFrames);
		assertEquals(900L, h.staleAfterMs);
	}

	@Test
	public void zeroHysteresisAndZeroStaleAreAllowed() {
		new HiveDetectionConfig(40.0, 0.0, 15.0, 80.0, 1, 1, 0L);
	}

	@Test
	public void rejectsNegativeHysteresis() {
		assertRejected(40.0, -0.1, 15.0, 80.0, 1, 3, 1500L);
	}

	@Test
	public void rejectsConfirmFramesBelowOne() {
		assertRejected(40.0, 2.0, 15.0, 80.0, 1, 0, 1500L);
	}

	@Test
	public void rejectsMinTagsBelowOne() {
		assertRejected(40.0, 2.0, 15.0, 80.0, 0, 3, 1500L);
	}

	@Test
	public void rejectsMinPlausibleNotBelowMax() {
		assertRejected(40.0, 2.0, 80.0, 80.0, 1, 3, 1500L);
		assertRejected(40.0, 2.0, 90.0, 80.0, 1, 3, 1500L);
	}

	@Test
	public void rejectsNegativeStaleAfter() {
		assertRejected(40.0, 2.0, 15.0, 80.0, 1, 3, -1L);
	}

	@Test
	public void rejectsNonFiniteDoubles() {
		double[] bad = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
		for (double v : bad) {
			assertRejected(v, 2.0, 15.0, 80.0, 1, 3, 1500L);
			assertRejected(40.0, v, 15.0, 80.0, 1, 3, 1500L);
			assertRejected(40.0, 2.0, v, 80.0, 1, 3, 1500L);
			assertRejected(40.0, 2.0, 15.0, v, 1, 3, 1500L);
		}
	}

	@Test
	public void withersValidate() {
		HiveDetectionConfig base = HiveDetectionConfig.defaults();
		try {
			base.withHysteresisIn(-1.0);
			fail("expected IllegalArgumentException");
		} catch (IllegalArgumentException expected) {
			// ok
		}
		try {
			base.withRaisedThresholdIn(Double.NaN);
			fail("expected IllegalArgumentException");
		} catch (IllegalArgumentException expected) {
			// ok
		}
	}

	private static void assertRejected(double thr, double hyst, double minH, double maxH,
			int minTags, int confirm, long stale) {
		try {
			new HiveDetectionConfig(thr, hyst, minH, maxH, minTags, confirm, stale);
			fail("expected IllegalArgumentException for " + thr + "," + hyst + "," + minH + ","
					+ maxH + "," + minTags + "," + confirm + "," + stale);
		} catch (IllegalArgumentException expected) {
			// ok
		}
	}
}
