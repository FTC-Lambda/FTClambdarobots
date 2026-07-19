package org.firstinspires.ftc.teamcode.vision;

import org.junit.Test;

import java.util.Collections;
import java.util.Optional;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class TargetPersistenceTest {

	private static BallGroup group(double tx, double ty, int size) {
		BallDetection d = new BallDetection(
				"green", 80, tx, ty, 2.0, Collections.emptyList(),
				1.0, 1000L, 160, 120, 40, 40, true);
		return BallGrouping.buildGroup(Collections.nCopies(size, d), null);
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
	public void doesNotReuseSameFrameAsNewDetection() {
		TargetPersistence p = new TargetPersistence();
		p.update(1000L, 5.0, Optional.of(group(0, 0, 1)));
		BallTarget again = p.update(1050L, 5.0, Optional.of(group(1, 0, 1)));
		assertTrue(again.isValid());
		assertFalse(again.isFresh());
	}
}
