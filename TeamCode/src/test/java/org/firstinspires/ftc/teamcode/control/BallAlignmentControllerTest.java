package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.vision.BallTarget;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class BallAlignmentControllerTest {

	@Test
	public void hysteresisStopsAtTwoAndRestartsAboveFour() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		assertTrue(c.update(target(8.0, true, 0), 1000, 0.02).isCorrectionActive());

		BallAlignmentController.Result centered =
				c.update(target(2.0, true, 0), 1050, 0.02);
		assertEquals(0.0, centered.getAppliedTurn(), 1e-9);
		assertFalse(centered.isStaticFrictionApplied());
		assertFalse(c.update(target(3.5, true, 0), 1100, 0.02).isCorrectionActive());
		assertTrue(c.update(target(4.1, true, 0), 1150, 0.02).isCorrectionActive());
	}

	@Test
	public void commandExpiresAfterOneHundredMilliseconds() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		c.update(target(12.0, true, 0), 1000, 0.02);

		BallAlignmentController.Result held =
				c.update(target(12.0, false, 80), 1080, 0.02);
		BallAlignmentController.Result expired =
				c.update(target(12.0, false, 101), 1101, 0.02);

		assertEquals(BallAlignmentController.Action.HOLDING_COMMAND, held.getAction());
		assertEquals(BallAlignmentController.Action.COMMAND_EXPIRED, expired.getAction());
		assertTrue(Math.abs(expired.getAppliedTurn()) <= Math.abs(held.getAppliedTurn()));
	}

	@Test
	public void invalidTargetStopsImmediatelyAndResetClearsState() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		c.update(target(12.0, true, 0), 1000, 0.02);
		assertEquals(0.0, c.update(BallTarget.none(), 1020, 0.02).getAppliedTurn(), 1e-9);

		c.reset();

		assertEquals(0.0, c.update(BallTarget.none(), 1040, 0.02).getAppliedTurn(), 1e-9);
	}

	@Test
	public void oppositeFreshCommandStopsBeforeReversing() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		assertTrue(c.update(target(12.0, true, 0), 1000, 0.02).getAppliedTurn() > 0);

		BallAlignmentController.Result guarded =
				c.update(target(-12.0, true, 0), 1050, 0.02);

		assertEquals(BallAlignmentController.Action.REVERSAL_GUARD, guarded.getAction());
		assertEquals(0.0, guarded.getAppliedTurn(), 1e-9);
		assertTrue(c.update(target(-12.0, true, 0), 1100, 0.02).getAppliedTurn() < 0);
	}

	@Test
	public void heldFrameCannotConfirmPendingReversal() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		c.update(target(12.0, true, 0), 1000, 0.02);
		c.update(target(-12.0, true, 0), 1050, 0.02);

		BallAlignmentController.Result held =
				c.update(target(-12.0, false, 20), 1070, 0.02);

		assertEquals(BallAlignmentController.Action.REVERSAL_GUARD, held.getAction());
		assertEquals(0.0, held.getAppliedTurn(), 1e-9);
		assertTrue(c.update(target(-12.0, true, 0), 1100, 0.02).getAppliedTurn() < 0);
	}

	@Test
	public void zeroFreshRequestDoesNotConfirmPendingReversal() {
		BallAlignmentConfig config = BallAlignmentConfig.defaults()
				.withTurnKp(0.125)
				.withTurnKi(1.0)
				.withTurnKd(0.0);
		BallAlignmentController c = new BallAlignmentController(config);
		assertTrue(c.update(target(8.0, true, 0), 1000, 0.125).getAppliedTurn() > 0);
		assertEquals(BallAlignmentController.Action.REVERSAL_GUARD,
				c.update(target(-16.0, true, 0), 1125, 0.125).getAction());

		BallAlignmentController.Result zero =
				c.update(target(4.0, true, 0), 1250, 0.125);

		assertEquals(0.0, zero.getRequestedTurn(), 1e-9);
		assertEquals(0.0, zero.getAppliedTurn(), 1e-9);
		assertEquals(BallAlignmentController.Action.REVERSAL_GUARD, zero.getAction());
		assertTrue(c.update(target(-8.0, true, 0), 1375, 0.125).getAppliedTurn() < 0);
	}

	@Test
	public void returnToOriginalSignCancelsPendingReversal() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		assertTrue(c.update(target(12.0, true, 0), 1000, 0.02).getAppliedTurn() > 0);
		assertEquals(BallAlignmentController.Action.REVERSAL_GUARD,
				c.update(target(-12.0, true, 0), 1050, 0.02).getAction());

		BallAlignmentController.Result resumed =
				c.update(target(12.0, true, 0), 1100, 0.02);

		assertEquals(BallAlignmentController.Action.CORRECTING, resumed.getAction());
		assertTrue(resumed.getAppliedTurn() > 0);
		BallAlignmentController.Result guardedAgain =
				c.update(target(-12.0, true, 0), 1150, 0.02);
		assertEquals(BallAlignmentController.Action.REVERSAL_GUARD, guardedAgain.getAction());
		assertEquals(0.0, guardedAgain.getAppliedTurn(), 1e-9);
	}

	@Test
	public void outputIsClampedAndFirstMotionIsSlewLimitedFromZero() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());

		BallAlignmentController.Result first =
				c.update(target(1000.0, true, 0), 1000, 0.02);

		assertTrue(Math.abs(first.getRequestedTurn()) <= 0.65);
		assertTrue(Math.abs(first.getAppliedTurn()) <= 0.65);
		assertTrue(first.isSlewLimited());
		assertTrue(Math.abs(first.getAppliedTurn()) < Math.abs(first.getRequestedTurn()));
	}

	@Test
	public void heldFrameDoesNotAdvancePidTerms() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		BallAlignmentController.Result fresh =
				c.update(target(12.0, true, 0), 1000, 0.02);

		BallAlignmentController.Result held =
				c.update(target(7.0, false, 50), 1050, 0.02);

		assertEquals(fresh.getP(), held.getP(), 1e-9);
		assertEquals(fresh.getI(), held.getI(), 1e-9);
		assertEquals(fresh.getD(), held.getD(), 1e-9);
		assertEquals(fresh.getRawPid(), held.getRawPid(), 1e-9);
		assertEquals(fresh.getRequestedTurn(), held.getRequestedTurn(), 1e-9);
	}

	@Test
	public void invalidLoopTimeStopsImmediately() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		c.update(target(12.0, true, 0), 1000, 0.02);

		BallAlignmentController.Result nanDt =
				c.update(target(12.0, true, 0), 1020, Double.NaN);

		assertEquals(BallAlignmentController.Action.NO_TARGET, nanDt.getAction());
		assertEquals(0.0, nanDt.getAppliedTurn(), 1e-9);

		c.update(target(12.0, true, 0), 1040, 0.02);
		assertEquals(0.0,
				c.update(target(12.0, true, 0), 1060, Double.POSITIVE_INFINITY)
						.getAppliedTurn(),
				1e-9);
	}

	@Test
	public void nonMonotonicClockStopsImmediately() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		c.update(target(12.0, true, 0), 1000, 0.02);

		BallAlignmentController.Result stopped =
				c.update(target(12.0, true, 0), 1000, 0.02);

		assertEquals(BallAlignmentController.Action.NO_TARGET, stopped.getAction());
		assertEquals(0.0, stopped.getAppliedTurn(), 1e-9);
		assertFalse(stopped.isCorrectionActive());
	}

	@Test
	public void setConfigResetsHeldCommandAndOutput() {
		BallAlignmentController c = new BallAlignmentController(BallAlignmentConfig.defaults());
		c.update(target(12.0, true, 0), 1000, 0.02);

		c.setConfig(BallAlignmentConfig.defaults().withTurnKp(0.02));
		BallAlignmentController.Result afterConfig =
				c.update(target(12.0, false, 20), 1020, 0.02);

		assertEquals(0.0, afterConfig.getRequestedTurn(), 1e-9);
		assertEquals(0.0, afterConfig.getAppliedTurn(), 1e-9);
		assertFalse(afterConfig.isCorrectionActive());
	}

	@Test
	public void centeredEntryClearsResidualPidAndOutput() {
		BallAlignmentController c = new BallAlignmentController(
				BallAlignmentConfig.defaults().withTurnKi(0.01));
		c.update(target(8.0, true, 0), 1000, 0.02);

		BallAlignmentController.Result centered =
				c.update(target(1.5, true, 0), 1050, 0.02);

		assertEquals(BallAlignmentController.Action.CENTERED, centered.getAction());
		assertEquals(0.0, centered.getP(), 1e-9);
		assertEquals(0.0, centered.getI(), 1e-9);
		assertEquals(0.0, centered.getD(), 1e-9);
		assertEquals(0.0, centered.getRawPid(), 1e-9);
		assertEquals(0.0, centered.getRequestedTurn(), 1e-9);
		assertEquals(0.0, centered.getAppliedTurn(), 1e-9);
		assertFalse(centered.isStaticFrictionApplied());
		assertFalse(centered.isSlewLimited());
	}

	private static BallTarget target(double bearing, boolean fresh, long ageMs) {
		return new BallTarget(true, fresh, bearing, 0.0, 0.8, 3, ageMs,
				0.0, 3, 0, null);
	}
}
