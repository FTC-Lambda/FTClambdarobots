package org.firstinspires.ftc.teamcode.control;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class BallAlignmentConfigTest {

	@Test
	public void alignmentConfigClampsDependentBounds() {
		BallAlignmentConfig c = BallAlignmentConfig.defaults()
				.withMaxTurnPower(0.10)
				.withStaticFrictionPower(0.40)
				.withStartCorrectionDeg(1.0)
				.withStopCorrectionDeg(3.0);
		assertEquals(0.10, c.getMaxTurnPower(), 1e-9);
		assertEquals(0.10, c.getStaticFrictionPower(), 1e-9);
		assertTrue(c.getStopCorrectionDeg() <= c.getStartCorrectionDeg());
	}

	@Test
	public void nonFiniteAlignmentValuesReturnSafeBounds() {
		BallAlignmentConfig c = BallAlignmentConfig.defaults()
				.withTurnKp(Double.NaN)
				.withTurnSlewRate(Double.POSITIVE_INFINITY);
		assertTrue(Double.isFinite(c.getTurnKp()));
		assertTrue(Double.isFinite(c.getTurnSlewRate()));
		assertTrue(c.getTurnKp() >= 0.0);
		assertTrue(c.getTurnSlewRate() >= 0.0);
	}
}
