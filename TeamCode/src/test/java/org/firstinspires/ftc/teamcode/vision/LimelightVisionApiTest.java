package org.firstinspires.ftc.teamcode.vision;

import org.junit.Test;

import java.util.Optional;

import static org.junit.Assert.assertEquals;

public class LimelightVisionApiTest {

	@Test
	public void exposesTargetingConfigurationAndDiagnostics() throws Exception {
		assertEquals(void.class, LimelightVision.class
				.getMethod("setBallTargetingConfig", BallTargetingConfig.class).getReturnType());
		assertEquals(BallTargetingConfig.class, LimelightVision.class
				.getMethod("getBallTargetingConfig").getReturnType());
		assertEquals(Optional.class, LimelightVision.class
				.getMethod("getPendingBallGroup").getReturnType());
		assertEquals(int.class, LimelightVision.class
				.getMethod("getPendingBallConfirmationFrames").getReturnType());
		assertEquals(double.class, LimelightVision.class
				.getMethod("getRawLockedTxDeg").getReturnType());
	}
}
