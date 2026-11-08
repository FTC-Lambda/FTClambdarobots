package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

public class HiveStateEstimatorTest {

	// Defaults: threshold 40, hysteresis 2 -> RAISED >= 42, LOWERED <= 38, 3 confirm frames,
	// stale after 1500 ms, plausible heights [15, 80].
	private static final double HIGH = 46.0;
	private static final double LOW = 34.0;
	private static final double EPS = 1e-9;

	private HiveStateEstimator est;

	@Before
	public void setUp() {
		est = new HiveStateEstimator(Alliance.RED, HiveDetectionConfig.defaults());
	}

	private static HiveTagObservation tag(Alliance a, HiveCell cell, double h) {
		return new HiveTagObservation(cell == HiveCell.FAR ? 30 : 34, a, cell, h);
	}

	private static HiveTagObservation red(HiveCell cell, double h) {
		return tag(Alliance.RED, cell, h);
	}

	private static List<HiveTagObservation> list(HiveTagObservation... obs) {
		return Arrays.asList(obs);
	}

	private static List<HiveTagObservation> cluster(Alliance a, HiveCell cell, double... hs) {
		List<HiveTagObservation> out = new ArrayList<>();
		for (double h : hs) {
			out.add(tag(a, cell, h));
		}
		return out;
	}

	private static List<HiveTagObservation> farRaised() {
		return list(red(HiveCell.FAR, HIGH));
	}

	private static List<HiveTagObservation> audienceRaised() {
		return list(red(HiveCell.AUDIENCE, HIGH));
	}

	private static List<HiveTagObservation> farLowered() {
		return list(red(HiveCell.FAR, LOW));
	}

	private static List<HiveTagObservation> audienceLowered() {
		return list(red(HiveCell.AUDIENCE, LOW));
	}

	// ---- HiveState / enums -------------------------------------------------------------

	@Test
	public void hiveStateHelpers() {
		assertEquals(HiveCell.FAR, HiveState.FAR_CELL_UP.upCell());
		assertEquals(HiveCell.AUDIENCE, HiveState.AUDIENCE_CELL_UP.upCell());
		assertNull(HiveState.UNKNOWN.upCell());
		assertEquals(HiveState.FAR_CELL_UP, HiveState.withUp(HiveCell.FAR));
		assertEquals(HiveState.AUDIENCE_CELL_UP, HiveState.withUp(HiveCell.AUDIENCE));
	}

	@Test
	public void observationGetters() {
		HiveTagObservation o = new HiveTagObservation(31, Alliance.RED, HiveCell.FAR, 44.5);
		assertEquals(31, o.getTagId());
		assertEquals(Alliance.RED, o.getAlliance());
		assertEquals(HiveCell.FAR, o.getCell());
		assertEquals(44.5, o.getHeightIn(), EPS);
	}

	// ---- Initial state -----------------------------------------------------------------

	@Test
	public void startsUnknown() {
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertFalse(est.isFresh(0L));
		assertEquals(-1L, est.getAgeMs(12345L));
		assertEquals(HiveState.UNKNOWN, est.getPendingState());
		assertEquals(0, est.getPendingFrames());
		assertEquals(Alliance.RED, est.getAlliance());
	}

	@Test
	public void emptyAndNullFramesDoNothing() {
		assertEquals(HiveState.UNKNOWN, est.update(0L, Collections.<HiveTagObservation>emptyList()));
		assertEquals(HiveState.UNKNOWN, est.update(10L, null));
		assertEquals(0, est.getPendingFrames());
	}

	// ---- First confirmation ------------------------------------------------------------

	@Test
	public void farRaisedNeedsExactlyConfirmFrames() {
		assertEquals(HiveState.UNKNOWN, est.update(100L, farRaised()));
		assertEquals(HiveState.FAR_CELL_UP, est.getPendingState());
		assertEquals(1, est.getPendingFrames());
		assertEquals(HiveState.UNKNOWN, est.update(200L, farRaised()));
		assertEquals(2, est.getPendingFrames());
		assertEquals(HiveState.FAR_CELL_UP, est.update(300L, farRaised()));
		assertEquals(HiveState.UNKNOWN, est.getPendingState());
		assertEquals(0, est.getPendingFrames());
		assertEquals(HiveState.FAR_CELL_UP, est.getState());
	}

	@Test
	public void audienceLoweredMeansFarUp() {
		est.update(100L, audienceLowered());
		est.update(200L, audienceLowered());
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(HiveState.FAR_CELL_UP, est.update(300L, audienceLowered()));
	}

	@Test
	public void audienceRaisedMeansAudienceUp() {
		est.update(100L, audienceRaised());
		est.update(200L, audienceRaised());
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(HiveState.AUDIENCE_CELL_UP, est.update(300L, audienceRaised()));
	}

	@Test
	public void farLoweredMeansAudienceUp() {
		est.update(100L, farLowered());
		est.update(200L, farLowered());
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(HiveState.AUDIENCE_CELL_UP, est.update(300L, farLowered()));
	}

	@Test
	public void bothCellsAgreeingConfirmsInConfirmFrames() {
		List<HiveTagObservation> frame = new ArrayList<>();
		frame.addAll(cluster(Alliance.RED, HiveCell.FAR, HIGH, HIGH + 1));
		frame.addAll(cluster(Alliance.RED, HiveCell.AUDIENCE, LOW, LOW - 1));
		est.update(100L, frame);
		est.update(200L, frame);
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(HiveState.FAR_CELL_UP, est.update(300L, frame));

		HiveStateEstimator other = new HiveStateEstimator(Alliance.RED, HiveDetectionConfig.defaults());
		List<HiveTagObservation> flipped = new ArrayList<>();
		flipped.addAll(cluster(Alliance.RED, HiveCell.FAR, LOW, LOW));
		flipped.addAll(cluster(Alliance.RED, HiveCell.AUDIENCE, HIGH, HIGH));
		other.update(100L, flipped);
		other.update(200L, flipped);
		assertEquals(HiveState.UNKNOWN, other.getState());
		assertEquals(HiveState.AUDIENCE_CELL_UP, other.update(300L, flipped));
	}

	@Test
	public void confirmFramesOneConfirmsImmediately() {
		HiveDetectionConfig one = new HiveDetectionConfig(40.0, 2.0, 15.0, 80.0, 1, 1, 1500L);
		HiveStateEstimator fast = new HiveStateEstimator(Alliance.RED, one);
		assertEquals(HiveState.FAR_CELL_UP, fast.update(5L, farRaised()));
	}

	// ---- Classification ----------------------------------------------------------------

	@Test
	public void ambiguousBandProducesNoVote() {
		// Inside (38, 42) is ambiguous; exact edges are RAISED/LOWERED.
		for (int i = 0; i < 5; i++) {
			est.update(100L * i, list(red(HiveCell.FAR, 40.0)));
		}
		assertEquals(CellPosition.AMBIGUOUS, est.getCellPosition(HiveCell.FAR));
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(HiveState.UNKNOWN, est.getPendingState());
		assertEquals(0, est.getPendingFrames());
	}

	@Test
	public void hysteresisEdgesAreInclusive() {
		est.update(0L, list(red(HiveCell.FAR, 42.0)));
		assertEquals(CellPosition.RAISED, est.getCellPosition(HiveCell.FAR));
		est.update(0L, list(red(HiveCell.FAR, 41.99)));
		assertEquals(CellPosition.AMBIGUOUS, est.getCellPosition(HiveCell.FAR));
		est.update(0L, list(red(HiveCell.FAR, 38.01)));
		assertEquals(CellPosition.AMBIGUOUS, est.getCellPosition(HiveCell.FAR));
		est.update(0L, list(red(HiveCell.FAR, 38.0)));
		assertEquals(CellPosition.LOWERED, est.getCellPosition(HiveCell.FAR));
	}

	@Test
	public void ambiguousCellDoesNotBlockOtherCellsVote() {
		List<HiveTagObservation> frame = new ArrayList<>();
		frame.add(red(HiveCell.FAR, 40.0));
		frame.add(red(HiveCell.AUDIENCE, HIGH));
		est.update(0L, frame);
		est.update(1L, frame);
		assertEquals(HiveState.AUDIENCE_CELL_UP, est.update(2L, frame));
	}

	@Test
	public void contradictoryCellsProduceNoVote() {
		// FAR raised -> FAR_UP, AUDIENCE raised -> AUDIENCE_UP: contradiction.
		List<HiveTagObservation> bothRaised = new ArrayList<>();
		bothRaised.add(red(HiveCell.FAR, HIGH));
		bothRaised.add(red(HiveCell.AUDIENCE, HIGH));
		// Both lowered is contradictory the other way round.
		List<HiveTagObservation> bothLowered = new ArrayList<>();
		bothLowered.add(red(HiveCell.FAR, LOW));
		bothLowered.add(red(HiveCell.AUDIENCE, LOW));
		for (int i = 0; i < 6; i++) {
			est.update(i, i % 2 == 0 ? bothRaised : bothLowered);
		}
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(HiveState.UNKNOWN, est.getPendingState());
		assertEquals(0, est.getPendingFrames());
		assertEquals(CellPosition.LOWERED, est.getCellPosition(HiveCell.FAR));
		assertEquals(CellPosition.LOWERED, est.getCellPosition(HiveCell.AUDIENCE));
	}

	@Test
	public void contradictoryFrameDoesNotAdvanceOrBreakPending() {
		est.update(0L, farRaised());
		List<HiveTagObservation> bothRaised = new ArrayList<>();
		bothRaised.add(red(HiveCell.FAR, HIGH));
		bothRaised.add(red(HiveCell.AUDIENCE, HIGH));
		est.update(1L, bothRaised);
		assertEquals(1, est.getPendingFrames());
		assertEquals(HiveState.FAR_CELL_UP, est.getPendingState());
	}

	// ---- Filtering ---------------------------------------------------------------------

	@Test
	public void otherAllianceObservationsAreIgnored() {
		List<HiveTagObservation> frame = new ArrayList<>();
		frame.add(tag(Alliance.BLUE, HiveCell.FAR, HIGH));
		frame.add(tag(Alliance.BLUE, HiveCell.AUDIENCE, HIGH));
		for (int i = 0; i < 5; i++) {
			est.update(i, frame);
		}
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(0, est.getCellTagCount(HiveCell.FAR));
		assertEquals(0, est.getCellTagCount(HiveCell.AUDIENCE));
		assertEquals(CellPosition.NOT_SEEN, est.getCellPosition(HiveCell.FAR));

		// Other-alliance tags mixed in must not skew our median either.
		List<HiveTagObservation> mixed = new ArrayList<>();
		mixed.add(red(HiveCell.FAR, HIGH));
		mixed.add(tag(Alliance.BLUE, HiveCell.FAR, LOW));
		mixed.add(tag(Alliance.BLUE, HiveCell.FAR, LOW));
		est.update(10L, mixed);
		assertEquals(1, est.getCellTagCount(HiveCell.FAR));
		assertEquals(HIGH, est.getCellHeightIn(HiveCell.FAR), EPS);
	}

	@Test
	public void blueEstimatorUsesBlueTags() {
		HiveStateEstimator blue = new HiveStateEstimator(Alliance.BLUE, HiveDetectionConfig.defaults());
		List<HiveTagObservation> frame = new ArrayList<>();
		frame.add(tag(Alliance.RED, HiveCell.FAR, LOW));
		frame.add(tag(Alliance.BLUE, HiveCell.FAR, HIGH));
		blue.update(0L, frame);
		blue.update(1L, frame);
		assertEquals(HiveState.FAR_CELL_UP, blue.update(2L, frame));
	}

	@Test
	public void implausibleAndNonFiniteHeightsAreIgnored() {
		List<HiveTagObservation> frame = new ArrayList<>();
		frame.add(red(HiveCell.FAR, 14.9));
		frame.add(red(HiveCell.FAR, 80.1));
		frame.add(red(HiveCell.FAR, Double.NaN));
		frame.add(red(HiveCell.FAR, Double.POSITIVE_INFINITY));
		frame.add(red(HiveCell.FAR, Double.NEGATIVE_INFINITY));
		frame.add(red(HiveCell.FAR, -5.0));
		est.update(0L, frame);
		assertEquals(0, est.getCellTagCount(HiveCell.FAR));
		assertEquals(CellPosition.NOT_SEEN, est.getCellPosition(HiveCell.FAR));
		assertTrue(Double.isNaN(est.getCellHeightIn(HiveCell.FAR)));
		assertEquals(0, est.getPendingFrames());

		// Plausible bounds are inclusive.
		frame = new ArrayList<>();
		frame.add(red(HiveCell.FAR, 15.0));
		frame.add(red(HiveCell.AUDIENCE, 80.0));
		est.update(1L, frame);
		assertEquals(1, est.getCellTagCount(HiveCell.FAR));
		assertEquals(1, est.getCellTagCount(HiveCell.AUDIENCE));
	}

	@Test
	public void implausibleTagsDoNotSkewMedian() {
		List<HiveTagObservation> frame = new ArrayList<>();
		frame.addAll(cluster(Alliance.RED, HiveCell.FAR, 45.0, 46.0, 47.0, 200.0, Double.NaN));
		est.update(0L, frame);
		assertEquals(3, est.getCellTagCount(HiveCell.FAR));
		assertEquals(46.0, est.getCellHeightIn(HiveCell.FAR), EPS);
	}

	@Test
	public void nullEntriesAndNullCellAreTolerated() {
		List<HiveTagObservation> frame = new ArrayList<>();
		frame.add(null);
		frame.add(new HiveTagObservation(99, Alliance.RED, null, HIGH));
		frame.add(red(HiveCell.FAR, HIGH));
		est.update(0L, frame);
		assertEquals(1, est.getCellTagCount(HiveCell.FAR));
	}

	@Test
	public void medianIsRobustToOneOutlierAmongFourTags() {
		// Three tags near 46 and one low outlier at 20 (plausible but wrong).
		List<HiveTagObservation> frame =
				cluster(Alliance.RED, HiveCell.FAR, 45.0, 46.0, 47.0, 20.0);
		est.update(0L, frame);
		assertEquals(4, est.getCellTagCount(HiveCell.FAR));
		// sorted 20,45,46,47 -> mean of 45 and 46
		assertEquals(45.5, est.getCellHeightIn(HiveCell.FAR), EPS);
		assertEquals(CellPosition.RAISED, est.getCellPosition(HiveCell.FAR));

		// A high outlier among low tags must not flip a lowered CELL either.
		frame = cluster(Alliance.RED, HiveCell.AUDIENCE, 33.0, 34.0, 35.0, 79.0);
		est.update(1L, frame);
		assertEquals(34.5, est.getCellHeightIn(HiveCell.AUDIENCE), EPS);
		assertEquals(CellPosition.LOWERED, est.getCellPosition(HiveCell.AUDIENCE));
	}

	@Test
	public void medianOddAndEvenCounts() {
		est.update(0L, cluster(Alliance.RED, HiveCell.FAR, 50.0, 30.0, 44.0));
		assertEquals(44.0, est.getCellHeightIn(HiveCell.FAR), EPS);
		est.update(1L, cluster(Alliance.RED, HiveCell.FAR, 50.0, 30.0));
		assertEquals(40.0, est.getCellHeightIn(HiveCell.FAR), EPS);
		est.update(2L, cluster(Alliance.RED, HiveCell.FAR, 48.0));
		assertEquals(48.0, est.getCellHeightIn(HiveCell.FAR), EPS);
	}

	@Test
	public void minTagsPerCellIsEnforced() {
		HiveDetectionConfig cfg = new HiveDetectionConfig(40.0, 2.0, 15.0, 80.0, 2, 1, 1500L);
		HiveStateEstimator strict = new HiveStateEstimator(Alliance.RED, cfg);
		assertEquals(HiveState.UNKNOWN, strict.update(0L, farRaised()));
		assertEquals(CellPosition.NOT_SEEN, strict.getCellPosition(HiveCell.FAR));
		assertEquals(1, strict.getCellTagCount(HiveCell.FAR));
		assertTrue(Double.isNaN(strict.getCellHeightIn(HiveCell.FAR)));
		assertEquals(HiveState.FAR_CELL_UP,
				strict.update(1L, cluster(Alliance.RED, HiveCell.FAR, HIGH, HIGH)));
	}

	// ---- Streak semantics --------------------------------------------------------------

	@Test
	public void nonVotingFramesDoNotBreakPendingStreak() {
		est.update(100L, farRaised());
		est.update(200L, Collections.<HiveTagObservation>emptyList());
		assertEquals(1, est.getPendingFrames());
		est.update(300L, list(red(HiveCell.FAR, 40.0))); // ambiguous
		est.update(400L, list(tag(Alliance.BLUE, HiveCell.FAR, LOW))); // other alliance
		assertEquals(1, est.getPendingFrames());
		assertEquals(HiveState.FAR_CELL_UP, est.getPendingState());
		est.update(500L, farRaised());
		assertEquals(2, est.getPendingFrames());
		assertEquals(HiveState.FAR_CELL_UP, est.update(600L, farRaised()));
	}

	@Test
	public void contradictingVoteRestartsStreak() {
		est.update(100L, farRaised());
		est.update(200L, farRaised());
		assertEquals(2, est.getPendingFrames());
		// One AUDIENCE_UP vote replaces the pending FAR_UP at count 1.
		est.update(300L, audienceRaised());
		assertEquals(HiveState.AUDIENCE_CELL_UP, est.getPendingState());
		assertEquals(1, est.getPendingFrames());
		assertEquals(HiveState.UNKNOWN, est.getState());
		// FAR vote again restarts at 1, so two more are not enough.
		est.update(400L, farRaised());
		est.update(500L, farRaised());
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(HiveState.FAR_CELL_UP, est.update(600L, farRaised()));
	}

	private void confirmFar(long startMs) {
		est.update(startMs, farRaised());
		est.update(startMs + 10, farRaised());
		est.update(startMs + 20, farRaised());
		assertEquals(HiveState.FAR_CELL_UP, est.getState());
	}

	@Test
	public void switchingConfirmedStateNeedsConfirmFramesAndKeepsOldStateMeanwhile() {
		confirmFar(0L);
		assertEquals(HiveState.FAR_CELL_UP, est.update(100L, audienceRaised()));
		assertEquals(HiveState.AUDIENCE_CELL_UP, est.getPendingState());
		assertEquals(HiveState.FAR_CELL_UP, est.update(110L, audienceRaised()));
		assertEquals(HiveState.AUDIENCE_CELL_UP, est.update(120L, audienceRaised()));
		assertEquals(0, est.getPendingFrames());
		assertEquals(HiveState.UNKNOWN, est.getPendingState());
	}

	@Test
	public void voteEqualToConfirmedRefreshesSupportAndClearsPending() {
		confirmFar(0L); // lastSupport = 20
		est.update(1000L, audienceRaised());
		est.update(1010L, audienceRaised());
		assertEquals(2, est.getPendingFrames());
		// Back to the confirmed state: pending cleared, support refreshed.
		est.update(1020L, farRaised());
		assertEquals(0, est.getPendingFrames());
		assertEquals(HiveState.UNKNOWN, est.getPendingState());
		assertEquals(0L, est.getAgeMs(1020L));
		// Pending counter was reset: two more AUDIENCE votes are not enough.
		est.update(1030L, audienceRaised());
		est.update(1040L, audienceRaised());
		assertEquals(HiveState.FAR_CELL_UP, est.getState());
	}

	@Test
	public void nonVotingFrameDoesNotRefreshSupport() {
		confirmFar(0L); // lastSupport = 20
		est.update(500L, Collections.<HiveTagObservation>emptyList());
		assertEquals(480L, est.getAgeMs(500L));
	}

	// ---- Staleness / age ---------------------------------------------------------------

	@Test
	public void stateSurvivesStalenessButIsNoLongerFresh() {
		confirmFar(0L); // lastSupport = 20, stale after 1500
		assertTrue(est.isFresh(20L));
		assertTrue(est.isFresh(1520L)); // exactly at the limit is still fresh
		assertFalse(est.isFresh(1521L));
		assertEquals(HiveState.FAR_CELL_UP, est.getState());
		// Empty frames long after the last support do not clear the state either.
		assertEquals(HiveState.FAR_CELL_UP,
				est.update(60000L, Collections.<HiveTagObservation>emptyList()));
		assertFalse(est.isFresh(60000L));
		assertEquals(HiveState.FAR_CELL_UP, est.getState());
		// A supporting frame makes it fresh again.
		est.update(61000L, farRaised());
		assertTrue(est.isFresh(61000L));
	}

	@Test
	public void ageMs() {
		assertEquals(-1L, est.getAgeMs(0L));
		confirmFar(100L); // confirmed at 120
		assertEquals(0L, est.getAgeMs(120L));
		assertEquals(380L, est.getAgeMs(500L));
		est.update(900L, farRaised());
		assertEquals(100L, est.getAgeMs(1000L));
	}

	@Test
	public void customStaleAfter() {
		HiveDetectionConfig cfg = new HiveDetectionConfig(40.0, 2.0, 15.0, 80.0, 1, 1, 100L);
		HiveStateEstimator e = new HiveStateEstimator(Alliance.RED, cfg);
		e.update(1000L, farRaised());
		assertTrue(e.isFresh(1100L));
		assertFalse(e.isFresh(1101L));
	}

	// ---- reset / setAlliance / setConfig -----------------------------------------------

	@Test
	public void resetClearsEverything() {
		confirmFar(0L);
		est.update(100L, audienceRaised());
		est.reset();
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(HiveState.UNKNOWN, est.getPendingState());
		assertEquals(0, est.getPendingFrames());
		assertFalse(est.isFresh(100L));
		assertEquals(-1L, est.getAgeMs(100L));
		assertEquals(CellPosition.NOT_SEEN, est.getCellPosition(HiveCell.FAR));
		assertEquals(CellPosition.NOT_SEEN, est.getCellPosition(HiveCell.AUDIENCE));
		assertTrue(Double.isNaN(est.getCellHeightIn(HiveCell.FAR)));
		assertEquals(0, est.getCellTagCount(HiveCell.AUDIENCE));
		// Alliance and config are kept by reset().
		assertEquals(Alliance.RED, est.getAlliance());
	}

	@Test
	public void setAllianceResetsAndSwitchesAlliance() {
		confirmFar(0L);
		est.update(100L, audienceRaised());
		est.setAlliance(Alliance.BLUE);
		assertEquals(Alliance.BLUE, est.getAlliance());
		assertEquals(HiveState.UNKNOWN, est.getState());
		assertEquals(0, est.getPendingFrames());
		assertEquals(-1L, est.getAgeMs(200L));
		// Now red tags are ignored and blue tags count.
		est.update(200L, farRaised());
		assertEquals(0, est.getCellTagCount(HiveCell.FAR));
		est.update(210L, list(tag(Alliance.BLUE, HiveCell.FAR, HIGH)));
		assertEquals(1, est.getCellTagCount(HiveCell.FAR));
		assertEquals(1, est.getPendingFrames());
	}

	@Test
	public void setConfigKeepsConfirmedStateButClearsPending() {
		confirmFar(0L);
		est.update(100L, audienceRaised());
		est.update(110L, audienceRaised());
		assertEquals(2, est.getPendingFrames());
		HiveDetectionConfig cfg = HiveDetectionConfig.defaults().withRaisedThresholdIn(50.0);
		est.setConfig(cfg);
		assertSame(cfg, est.getConfig());
		assertEquals(HiveState.FAR_CELL_UP, est.getState());
		assertEquals(80L, est.getAgeMs(100L)); // lastSupport (20) is preserved
		assertEquals(HiveState.UNKNOWN, est.getPendingState());
		assertEquals(0, est.getPendingFrames());
	}

	@Test
	public void setConfigChangesClassification() {
		// With threshold 50, a 46 in cluster is now LOWERED (<= 48).
		est.setConfig(HiveDetectionConfig.defaults().withRaisedThresholdIn(50.0));
		est.update(0L, farRaised());
		assertEquals(CellPosition.LOWERED, est.getCellPosition(HiveCell.FAR));
	}

	@Test
	public void hysteresisConfigWidensAmbiguousBand() {
		est.setConfig(HiveDetectionConfig.defaults().withHysteresisIn(8.0));
		est.update(0L, farRaised()); // 46 < 48
		assertEquals(CellPosition.AMBIGUOUS, est.getCellPosition(HiveCell.FAR));
		est.update(1L, farLowered()); // 34 > 32
		assertEquals(CellPosition.AMBIGUOUS, est.getCellPosition(HiveCell.FAR));
	}

	@Test(expected = IllegalArgumentException.class)
	public void constructorRejectsNullAlliance() {
		new HiveStateEstimator(null, HiveDetectionConfig.defaults());
	}

	@Test(expected = IllegalArgumentException.class)
	public void constructorRejectsNullConfig() {
		new HiveStateEstimator(Alliance.RED, null);
	}

	// ---- Diagnostics -------------------------------------------------------------------

	@Test
	public void diagnosticsWhenNothingSeen() {
		est.update(0L, Collections.<HiveTagObservation>emptyList());
		for (HiveCell c : HiveCell.values()) {
			assertTrue(Double.isNaN(est.getCellHeightIn(c)));
			assertEquals(CellPosition.NOT_SEEN, est.getCellPosition(c));
			assertEquals(0, est.getCellTagCount(c));
		}
	}

	@Test
	public void diagnosticsReportLastUpdateOnly() {
		List<HiveTagObservation> frame = new ArrayList<>();
		frame.addAll(cluster(Alliance.RED, HiveCell.FAR, 45.0, 46.0, 47.0, 46.0));
		frame.addAll(cluster(Alliance.RED, HiveCell.AUDIENCE, 33.0, 35.0));
		est.update(0L, frame);
		assertEquals(4, est.getCellTagCount(HiveCell.FAR));
		assertEquals(2, est.getCellTagCount(HiveCell.AUDIENCE));
		assertEquals(46.0, est.getCellHeightIn(HiveCell.FAR), EPS);
		assertEquals(34.0, est.getCellHeightIn(HiveCell.AUDIENCE), EPS);
		assertEquals(CellPosition.RAISED, est.getCellPosition(HiveCell.FAR));
		assertEquals(CellPosition.LOWERED, est.getCellPosition(HiveCell.AUDIENCE));

		// Next frame sees only the FAR cell: AUDIENCE diagnostics go back to unseen.
		est.update(10L, farRaised());
		assertEquals(1, est.getCellTagCount(HiveCell.FAR));
		assertEquals(0, est.getCellTagCount(HiveCell.AUDIENCE));
		assertTrue(Double.isNaN(est.getCellHeightIn(HiveCell.AUDIENCE)));
		assertEquals(CellPosition.NOT_SEEN, est.getCellPosition(HiveCell.AUDIENCE));
	}
}
