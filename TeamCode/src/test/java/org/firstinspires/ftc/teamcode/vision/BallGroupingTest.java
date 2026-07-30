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

	/**
	 * Confidence and area are 0–1 fractions here because that is what the Limelight
	 * actually reports; measured hardware values are conf 0.7–0.8 and ta 0.01–0.02.
	 */
	private static BallDetection det(
			String cls, double conf, double tx, double ty, double area,
			double cx, double cy, double w, double h, boolean hasBox) {
		return new BallDetection(cls, conf, tx, ty, area, Collections.emptyList(),
				1.0, 1000L, cx, cy, w, h, hasBox);
	}

	@Test
	public void measuredHardwareValuesSurviveFiltering() {
		// Exactly what Test-Vision reported on the robot: two purple balls low in frame.
		BallDetection t1 = det("purple", 0.7, 11.55, -17.24, 0.02, 0, 0, 0, 0, false);
		BallDetection t2 = det("purple", 0.8, -17.45, -11.14, 0.01, 0, 0, 0, 0, false);
		List<BallDetection> kept = BallGrouping.filterDetections(Arrays.asList(t1, t2));
		assertEquals(2, kept.size());
	}

	@Test
	public void ballLowInFrameIsKept() {
		// A ball close to the intake sits low; clipping it would blind the robot on approach.
		BallDetection close = det("green", 0.8, 0, -22.0, 0.05, 0, 0, 0, 0, false);
		assertFalse(BallGrouping.isNearImageEdge(close));
	}

	@Test
	public void rejectCountsReportWhichGateFailed() {
		BallGrouping.RejectCounts counts = new BallGrouping.RejectCounts();
		BallDetection weak = det("green", 0.2, 0, 0, 0.02, 0, 0, 0, 0, false);
		BallDetection tiny = det("green", 0.8, 0, 0, 0.0001, 0, 0, 0, 0, false);
		BallDetection unknown = det("robot", 0.8, 0, 0, 0.02, 0, 0, 0, 0, false);
		BallDetection edge = det("green", 0.8, 30, 0, 0.02, 0, 0, 0, 0, false);
		assertTrue(BallGrouping.filterDetections(
				Arrays.asList(weak, tiny, unknown, edge), counts).isEmpty());
		assertEquals(1, counts.lowConfidence);
		assertEquals(1, counts.smallArea);
		assertEquals(1, counts.unknownColor);
		assertEquals(1, counts.nearEdge);
		assertEquals(4, counts.total());
	}

	@Test
	public void spanCapStopsChainFromSpanningFrame() {
		// Each neighbour is within the angular gap, but the chain must not merge into one
		// group whose mean would aim at empty floor between the end clusters.
		BallDetection a = det("green", 0.8, -14, 0, 0.02, 0, 0, 0, 0, false);
		BallDetection b = det("green", 0.8, -7, 0, 0.02, 0, 0, 0, 0, false);
		BallDetection c = det("green", 0.8, 0, 0, 0.02, 0, 0, 0, 0, false);
		BallDetection d = det("green", 0.8, 7, 0, 0.02, 0, 0, 0, 0, false);
		BallDetection e = det("green", 0.8, 14, 0, 0.02, 0, 0, 0, 0, false);
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(a, b, c, d, e));
		assertTrue(groups.size() > 1);
		for (BallGroup g : groups) {
			double min = Double.POSITIVE_INFINITY;
			double max = Double.NEGATIVE_INFINITY;
			for (BallDetection m : g.getMembers()) {
				min = Math.min(min, m.getTxDeg());
				max = Math.max(max, m.getTxDeg());
			}
			assertTrue(max - min <= BallVisionConfig.MAX_GROUP_SPAN_DEG);
		}
	}

	@Test
	public void wrongColourMemberPenalisesMixedGroup() {
		double pureGreen = BallGrouping.scoreGroup(
				0, 0, 2, 0.04, 0.8, 2, 0, BallColor.GREEN);
		double mixed = BallGrouping.scoreGroup(
				0, 0, 2, 0.04, 0.8, 1, 1, BallColor.GREEN);
		assertTrue(mixed < pureGreen);
	}

	@Test
	public void buildGroupRejectsEmptyMembers() {
		try {
			BallGrouping.buildGroup(Collections.<BallDetection>emptyList(), null);
			throw new AssertionError("expected IllegalArgumentException");
		} catch (IllegalArgumentException expected) {
			// expected
		}
	}

	@Test
	public void noDetections() {
		assertTrue(BallGrouping.filterDetections(Collections.emptyList()).isEmpty());
		assertTrue(BallGrouping.groupDetections(Collections.emptyList()).isEmpty());
		assertFalse(BallGrouping.selectBestGroup(Collections.emptyList(), null).isPresent());
	}

	@Test
	public void oneBall() {
		BallDetection a = det("green", 0.8, 2, 0, 0.02, 160, 120, 40, 40, true);
		List<BallGroup> groups = BallGrouping.groupDetections(BallGrouping.filterDetections(
				Collections.singletonList(a)));
		assertEquals(1, groups.size());
		assertEquals(1, groups.get(0).getSize());
		assertEquals(1, groups.get(0).getGreenCount());
	}

	@Test
	public void twoNearbyBallsFormOneGroup() {
		BallDetection a = det("green", 0.8, -2, 0, 0.02, 140, 120, 40, 40, true);
		BallDetection b = det("purple", 0.75, 2, 0, 0.02, 175, 120, 40, 40, true);
		assertTrue(BallGrouping.areNearby(a, b));
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(a, b));
		assertEquals(1, groups.size());
		assertEquals(2, groups.get(0).getSize());
	}

	@Test
	public void twoFarApartBallsFormTwoGroups() {
		BallDetection a = det("green", 0.8, -20, 0, 0.02, 40, 120, 30, 30, true);
		BallDetection b = det("purple", 0.75, 20, 0, 0.02, 280, 120, 30, 30, true);
		assertFalse(BallGrouping.areNearby(a, b));
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(a, b));
		assertEquals(2, groups.size());
	}

	@Test
	public void chainANearBNearCIsOneGroup() {
		BallDetection a = det("green", 0.8, -6, 0, 0.02, 100, 120, 40, 40, true);
		BallDetection b = det("green", 0.8, 0, 0, 0.02, 140, 120, 40, 40, true);
		BallDetection c = det("purple", 0.8, 6, 0, 0.02, 180, 120, 40, 40, true);
		assertTrue(BallGrouping.areNearby(a, b));
		assertTrue(BallGrouping.areNearby(b, c));
		assertFalse(BallGrouping.areNearby(a, c));
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(a, b, c));
		assertEquals(1, groups.size());
		assertEquals(3, groups.get(0).getSize());
	}

	@Test
	public void overlappingDetectionsGroup() {
		BallDetection a = det("green", 0.8, 0, 0, 0.03, 160, 120, 50, 50, true);
		BallDetection b = det("green", 0.7, 1, 0, 0.025, 165, 122, 48, 48, true);
		assertTrue(BallGrouping.areNearby(a, b));
		assertEquals(1, BallGrouping.groupDetections(Arrays.asList(a, b)).size());
	}

	@Test
	public void mixedColorCounts() {
		BallDetection g = det("green", 0.9, 0, 0, 0.02, 150, 120, 40, 40, true);
		BallDetection p = det("purple", 0.9, 3, 0, 0.02, 185, 120, 40, 40, true);
		BallGroup group = BallGrouping.groupDetections(Arrays.asList(g, p)).get(0);
		assertEquals(1, group.getGreenCount());
		assertEquals(1, group.getPurpleCount());
	}

	@Test
	public void lowConfidenceRejected() {
		BallDetection weak = det("green", 0.1, 0, 0, 0.02, 160, 120, 40, 40, true);
		assertTrue(BallGrouping.filterDetections(Collections.singletonList(weak)).isEmpty());
	}

	@Test
	public void edgeDetectionsRejected() {
		BallDetection edge = det("green", 0.9, 28, 0, 0.02, 5, 120, 20, 20, true);
		assertTrue(BallGrouping.isNearImageEdge(edge));
		assertTrue(BallGrouping.filterDetections(Collections.singletonList(edge)).isEmpty());
	}

	@Test
	public void moreBallsBeatCloserSmallerGroup() {
		BallGroup threeFar = BallGrouping.buildGroup(Arrays.asList(
				det("green", 0.8, 12, 0, 0.01, 0, 0, 0, 0, false),
				det("green", 0.8, 14, 0, 0.01, 0, 0, 0, 0, false),
				det("green", 0.8, 16, 0, 0.01, 0, 0, 0, 0, false)), null);
		BallGroup twoNear = BallGrouping.buildGroup(Arrays.asList(
				det("green", 0.8, -10, 0, 0.05, 0, 0, 0, 0, false),
				det("green", 0.8, -8, 0, 0.05, 0, 0, 0, 0, false)), null);
		assertEquals(threeFar,
				BallGrouping.selectPriorityGroup(Arrays.asList(twoNear, threeFar), null).get());
	}

	@Test
	public void equalCountUsesClosenessThenConfidenceThenCenter() {
		BallGroup farther = BallGrouping.buildGroup(
				Collections.singletonList(det("green", 0.99, 0, 0, 0.02, 0, 0, 0, 0, false)), null);
		BallGroup nearer = BallGrouping.buildGroup(
				Collections.singletonList(det("green", 0.60, 15, 0, 0.03, 0, 0, 0, 0, false)), null);
		assertTrue(BallGrouping.isHigherPriority(nearer, farther));
	}

	@Test
	public void desiredColorSelectionPrefersMatchingGroup() {
		BallDetection green = det("green", 0.9, 0, 0, 0.03, 160, 120, 40, 40, true);
		BallDetection purpleFar = det("purple", 0.9, 18, 0, 0.03, 260, 120, 40, 40, true);
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(green, purpleFar));
		Optional<BallGroup> bestPurple = BallGrouping.selectBestGroup(groups, BallColor.PURPLE);
		assertTrue(bestPurple.isPresent());
		assertEquals(1, bestPurple.get().getPurpleCount());
		assertEquals(0, bestPurple.get().getGreenCount());
	}

	@Test
	public void selectsCloserGroupByAreaNotCenterline() {
		// Small ball dead-center vs larger (nearer) ball off to the side — closest wins.
		BallDetection near = det("purple", 0.8, 15, -10, 0.05, 0, 0, 0, 0, false);
		BallDetection farCenter = det("purple", 0.8, 1, -5, 0.01, 0, 0, 0, 0, false);
		List<BallGroup> groups = BallGrouping.groupDetections(Arrays.asList(near, farCenter));
		assertEquals(2, groups.size());
		Optional<BallGroup> best = BallGrouping.selectClosestGroup(groups, null);
		assertTrue(best.isPresent());
		assertEquals(0.05, BallGrouping.apparentCloseness(best.get()), 1e-9);
		assertTrue(Math.abs(best.get().getAverageTxDeg() - 15.0) < 0.1);
	}

	@Test
	public void angularFallbackNearby() {
		BallDetection a = det("green", 0.8, 0, 0, 0.02, 0, 0, 0, 0, false);
		BallDetection b = det("purple", 0.8, 4, 2, 0.02, 0, 0, 0, 0, false);
		assertTrue(BallGrouping.areNearby(a, b));
	}

	@Test
	public void groupPropertiesWeightedCenter() {
		BallDetection small = det("green", 0.5, -10, 0, 0.01, 100, 120, 20, 20, true);
		BallDetection large = det("green", 0.9, 10, 0, 0.05, 200, 120, 60, 60, true);
		BallGroup g = BallGrouping.buildGroup(Arrays.asList(small, large), null);
		assertTrue(g.getWeightedTxDeg() > g.getAverageTxDeg());
		assertEquals(2, g.getSize());
	}
}
