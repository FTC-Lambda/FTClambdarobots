package org.firstinspires.ftc.teamcode.vision;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TargetPersistenceTest {

	private static BallGroup group(double tx, double ty, int size, double area) {
		List<BallDetection> members = new ArrayList<>();
		for (int i = 0; i < size; i++) {
			members.add(new BallDetection(
					"green", 0.8, tx, ty, area, Collections.emptyList(),
					1.0, 1000L + i, 160 + i, 120, 40, 40, true));
		}
		return BallGrouping.buildGroup(members, null);
	}

	private static BallGroup group(double tx, double ty, int size) {
		return group(tx, ty, size, 0.02);
	}

	@Test
	public void persistsAcrossFramesThenClearsAfterTimeout() {
		TargetPersistence p = new TargetPersistence();
		BallTarget t1 = p.update(1000L, 1.0, Optional.of(group(0, 0, 2)));
		assertTrue(t1.isValid());
		assertTrue(t1.isFresh());

		BallTarget held = p.update(1100L, Double.NaN, Optional.empty());
		assertTrue(held.isValid());
		assertFalse(held.isFresh());
		assertEquals(100L, held.getAgeMs());

		BallTarget gone = p.update(1000L + BallVisionConfig.VISION_LOSS_TIMEOUT_MS + 1,
				Double.NaN, Optional.empty());
		assertFalse(gone.isValid());
	}

	@Test
	public void associatesNearbyCenters() {
		BallGroup a = group(0, 0, 1);
		BallGroup b = group(5, 2, 1);
		BallGroup far = group(25, 0, 1);
		assertTrue(TargetPersistence.areAssociated(a, b));
		assertFalse(TargetPersistence.areAssociated(a, far));
	}

	@Test
	public void staysOnTrackedClusterWhenAreasAreSimilar() {
		TargetPersistence p = new TargetPersistence(
				BallTargetingConfig.defaults().withAimFilterGain(1.0));
		p.update(1000L, 1.0, Optional.of(group(0, 0, 1, 0.02)));

		// Drifted tracked pile vs another pile at similar size — stay locked.
		BallGroup drifted = group(3, 0, 1, 0.021);
		BallGroup other = group(20, 0, 1, 0.022);
		BallTarget next = p.update(1100L, 2.0, Arrays.asList(other, drifted));
		assertEquals(3.0, next.getHorizontalErrorDeg(), 1e-6);
	}

	@Test
	public void switchesWhenAnotherGroupIsClearlyCloser() {
		TargetPersistence p = new TargetPersistence();
		p.update(1000L, 1.0, Optional.of(group(20, 0, 1, 0.02)));

		BallGroup nearer = group(5, 0, 1, 0.05);
		p.update(1050L, 2.0, Arrays.asList(group(20, 0, 1, 0.02), nearer));
		p.update(1100L, 3.0, Arrays.asList(group(20, 0, 1, 0.02), nearer));
		BallTarget next = p.update(1150L, 4.0,
				Arrays.asList(group(20, 0, 1, 0.02), nearer));
		assertEquals(5.0, next.getHorizontalErrorDeg(), 1e-6);
		assertEquals(0.05, BallGrouping.apparentCloseness(p.getStickyGroup().get()), 1e-9);
	}

	@Test
	public void challengerRequiresThreeConsecutiveFreshFrames() {
		TargetPersistence p = new TargetPersistence(
				BallTargetingConfig.defaults().withSwitchConfirmFrames(3));
		BallGroup locked = group(15, 0, 1, 0.02);
		BallGroup challenger = group(-10, 0, 2, 0.03);
		assertEquals(15.0,
				p.update(1000, 1.0, Arrays.asList(locked)).getHorizontalErrorDeg(), 1e-6);
		assertEquals(15.0,
				p.update(1050, 2.0, Arrays.asList(locked, challenger))
						.getHorizontalErrorDeg(),
				1e-6);
		assertEquals(1, p.getPendingConfirmationFrames());
		assertEquals(15.0,
				p.update(1100, 3.0, Arrays.asList(locked, challenger))
						.getHorizontalErrorDeg(),
				1e-6);
		BallTarget switched = p.update(1150, 4.0, Arrays.asList(locked, challenger));
		assertEquals(-10.0, switched.getHorizontalErrorDeg(), 1e-6);
		assertEquals(0, p.getPendingConfirmationFrames());
	}

	@Test
	public void oneFrameCountJumpDoesNotSwitchLock() {
		TargetPersistence p = new TargetPersistence();
		BallGroup locked = group(0, 0, 2, 0.02);
		p.update(1000, 1.0, Arrays.asList(locked));
		BallGroup transientThree = group(18, 0, 3, 0.03);
		p.update(1050, 2.0, Arrays.asList(locked, transientThree));
		BallTarget recovered = p.update(1100, 3.0, Arrays.asList(locked));
		assertTrue(Math.abs(recovered.getHorizontalErrorDeg()) < 1.0);
		assertEquals(0, p.getPendingConfirmationFrames());
	}

	@Test
	public void aimpointUsesConfiguredExponentialFilter() {
		TargetPersistence p = new TargetPersistence(
				BallTargetingConfig.defaults().withAimFilterGain(0.50));
		p.update(1000, 1.0, Arrays.asList(group(0, 0, 2)));
		BallTarget next = p.update(1050, 2.0, Arrays.asList(group(4, 0, 2)));
		assertEquals(2.0, next.getHorizontalErrorDeg(), 1e-6);
		assertEquals(4.0, p.getRawLockedTxDeg(), 1e-6);
	}

	@Test
	public void missingLockHoldsOldTargetWhileCandidateConfirms() {
		TargetPersistence p = new TargetPersistence();
		p.update(1000, 1.0, Arrays.asList(group(20, 0, 2)));
		BallTarget pending = p.update(1050, 2.0, Arrays.asList(group(-15, 0, 3)));
		assertTrue(pending.isValid());
		assertFalse(pending.isFresh());
		assertEquals(50L, pending.getAgeMs());
	}

	@Test
	public void changingDesiredColorClearsLockAndPendingChallenger() {
		TargetPersistence p = new TargetPersistence();
		p.update(1000, 1.0, Arrays.asList(group(0, 0, 2)));
		p.update(1050, 2.0, Arrays.asList(group(0, 0, 2), group(18, 0, 3)));
		p.setDesiredColor(BallColor.PURPLE);
		assertFalse(p.getStickyGroup().isPresent());
		assertFalse(p.getPendingGroup().isPresent());
		assertEquals(0, p.getPendingConfirmationFrames());
	}

	@Test
	public void doesNotReuseSameFrameAsNewDetection() {
		TargetPersistence p = new TargetPersistence();
		p.update(1000L, 5.0, Optional.of(group(0, 0, 1)));
		BallTarget again = p.update(1050L, 5.0, Optional.of(group(1, 0, 1)));
		assertTrue(again.isValid());
		assertFalse(again.isFresh());
	}
}
