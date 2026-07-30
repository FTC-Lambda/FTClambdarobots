package org.firstinspires.ftc.teamcode.vision;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class BallTargetingConfigTest {

	@Test
	public void targetingConfigClampsFramesAndFilter() {
		BallTargetingConfig low = BallTargetingConfig.defaults()
				.withSwitchConfirmFrames(0)
				.withAimFilterGain(-1.0);
		BallTargetingConfig high = BallTargetingConfig.defaults()
				.withSwitchConfirmFrames(50)
				.withAimFilterGain(5.0);
		assertEquals(1, low.getSwitchConfirmFrames());
		assertTrue(low.getAimFilterGain() > 0.0);
		assertEquals(10, high.getSwitchConfirmFrames());
		assertEquals(1.0, high.getAimFilterGain(), 1e-9);
	}
}
