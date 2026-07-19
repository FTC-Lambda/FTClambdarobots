package org.firstinspires.ftc.teamcode.vision;

import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class BallGroupingTest {

	private static BallDetection det(
			String cls, double conf, double tx, double ty, double area,
			double cx, double cy, double w, double h, boolean hasBox) {
		return new BallDetection(cls, conf, tx, ty, area, Collections.emptyList(),
				1.0, 1000L, cx, cy, w, h, hasBox);
	}

	@Test
	public void noDetections() {
		assertTrue(BallGrouping.filterDetections(Collections.emptyList()).isEmpty());
		assertTrue(BallGrouping.groupDetections(Collections.emptyList()).isEmpty());
		assertFalse(BallGrouping.selectBestGroup(Collections.emptyList(), null).isPresent());
	}

	@Test
	public void oneBall() {
		BallDetection a = det("green", 80, 2, 0, 2.0, 160, 120, 40, 40, true);
		List<BallGroup> groups = BallGrouping.groupDetections(BallGrouping.filterDetections(
				Collections.singletonList(a)));
		assertEquals(1, groups.size());
		assertEquals(1, groups.get(0).getSize());
		assertEquals(1, groups.get(0).getGreenCount());
	}

	@Test
	public void twoNearbyBallsFormOneGroup() {
		BallDetection a = det("green", 80, -2, 0, 2.0, 140, 120, 40, 40, true);
		BallDetection b = det("purple", 75, 2, 0, 2.0, 175, 120, 40, 40, true);
		assertTrue(BallGrouping.areNearby(a, b));
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(a, b));
		assertEquals(1, groups.size());
		assertEquals(2, groups.get(0).getSize());
	}

	@Test
	public void twoFarApartBallsFormTwoGroups() {
		BallDetection a = det("green", 80, -20, 0, 2.0, 40, 120, 30, 30, true);
		BallDetection b = det("purple", 75, 20, 0, 2.0, 280, 120, 30, 30, true);
		assertFalse(BallGrouping.areNearby(a, b));
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(a, b));
		assertEquals(2, groups.size());
	}

	@Test
	public void chainANearBNearCIsOneGroup() {
		BallDetection a = det("green", 80, -6, 0, 2.0, 100, 120, 40, 40, true);
		BallDetection b = det("green", 80, 0, 0, 2.0, 140, 120, 40, 40, true);
		BallDetection c = det("purple", 80, 6, 0, 2.0, 180, 120, 40, 40, true);
		assertTrue(BallGrouping.areNearby(a, b));
		assertTrue(BallGrouping.areNearby(b, c));
		assertFalse(BallGrouping.areNearby(a, c));
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(a, b, c));
		assertEquals(1, groups.size());
		assertEquals(3, groups.get(0).getSize());
	}

	@Test
	public void overlappingDetectionsGroup() {
		BallDetection a = det("green", 80, 0, 0, 3.0, 160, 120, 50, 50, true);
		BallDetection b = det("green", 70, 1, 0, 2.5, 165, 122, 48, 48, true);
		assertTrue(BallGrouping.areNearby(a, b));
		assertEquals(1, BallGrouping.groupDetections(Arrays.asList(a, b)).size());
	}

	@Test
	public void mixedColorCounts() {
		BallDetection g = det("green", 90, 0, 0, 2.0, 150, 120, 40, 40, true);
		BallDetection p = det("purple", 90, 3, 0, 2.0, 185, 120, 40, 40, true);
		BallGroup group = BallGrouping.groupDetections(Arrays.asList(g, p)).get(0);
		assertEquals(1, group.getGreenCount());
		assertEquals(1, group.getPurpleCount());
	}

	@Test
	public void lowConfidenceRejected() {
		BallDetection weak = det("green", 10, 0, 0, 2.0, 160, 120, 40, 40, true);
		assertTrue(BallGrouping.filterDetections(Collections.singletonList(weak)).isEmpty());
	}

	@Test
	public void edgeDetectionsRejected() {
		BallDetection edge = det("green", 90, 28, 0, 2.0, 5, 120, 20, 20, true);
		assertTrue(BallGrouping.isNearImageEdge(edge));
		assertTrue(BallGrouping.filterDetections(Collections.singletonList(edge)).isEmpty());
	}

	@Test
	public void desiredColorSelectionPrefersMatchingGroup() {
		BallDetection green = det("green", 90, 0, 0, 3.0, 160, 120, 40, 40, true);
		BallDetection purpleFar = det("purple", 90, 18, 0, 3.0, 260, 120, 40, 40, true);
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(green, purpleFar));
		Optional<BallGroup> bestPurple = BallGrouping.selectBestGroup(groups, BallColor.PURPLE);
		assertTrue(bestPurple.isPresent());
		assertEquals(1, bestPurple.get().getPurpleCount());
		assertEquals(0, bestPurple.get().getGreenCount());
	}

	@Test
	public void angularFallbackNearby() {
		BallDetection a = det("green", 80, 0, 0, 2.0, 0, 0, 0, 0, false);
		BallDetection b = det("purple", 80, 4, 2, 2.0, 0, 0, 0, 0, false);
		assertTrue(BallGrouping.areNearby(a, b));
	}

	@Test
	public void groupPropertiesWeightedCenter() {
		BallDetection small = det("green", 50, -10, 0, 1.0, 100, 120, 20, 20, true);
		BallDetection large = det("green", 90, 10, 0, 5.0, 200, 120, 60, 60, true);
		BallGroup g = BallGrouping.buildGroup(Arrays.asList(small, large), null);
		assertTrue(g.getWeightedTxDeg() > g.getAverageTxDeg());
		assertEquals(2, g.getSize());
	}
}
