package org.firstinspires.ftc.teamcode.vision.hive;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class HiveAccuracyTallyTest {

	@Test
	public void ignoresFramesUntilTruthIsSet() {
		HiveAccuracyTally t = new HiveAccuracyTally();
		t.record(HiveState.FAR_CELL_UP, HiveState.FAR_CELL_UP, 4);
		assertEquals(0, t.getFrames());
		assertEquals("no frames scored", t.frameSummary());
	}

	@Test
	public void scoresFrameVotesAndConfirmedStateSeparately() {
		HiveAccuracyTally t = new HiveAccuracyTally();
		t.setTruth(HiveState.FAR_CELL_UP);
		t.record(HiveState.FAR_CELL_UP, HiveState.UNKNOWN, 4);        // right vote, not yet confirmed
		t.record(HiveState.AUDIENCE_CELL_UP, HiveState.UNKNOWN, 2);   // wrong vote
		t.record(HiveState.UNKNOWN, HiveState.FAR_CELL_UP, 0);        // nothing seen
		t.record(HiveState.FAR_CELL_UP, HiveState.FAR_CELL_UP, 3);
		assertEquals(4, t.getFrames());
		assertEquals(3, t.getFramesWithOwnTags());
		assertEquals(2, t.getVoteRight());
		assertEquals(1, t.getVoteWrong());
		assertEquals(1, t.getNoVote());
		assertEquals(2, t.getConfirmedRight());
		assertEquals(0, t.getConfirmedWrong());
		assertEquals(2, t.getConfirmedUnknown());
		assertEquals("right 50%  wrong 25%  no vote 25%  (seen 75% of 4 frames)", t.frameSummary());
		assertEquals("right 50%  wrong 0%  unknown 50%", t.confirmedSummary());
	}

	@Test
	public void changingTruthStartsANewTallyAndResetKeepsTruth() {
		HiveAccuracyTally t = new HiveAccuracyTally();
		t.setTruth(HiveState.FAR_CELL_UP);
		t.record(HiveState.FAR_CELL_UP, HiveState.FAR_CELL_UP, 4);
		t.setTruth(HiveState.AUDIENCE_CELL_UP);
		assertEquals(0, t.getFrames());
		t.record(HiveState.FAR_CELL_UP, HiveState.FAR_CELL_UP, 4);
		assertEquals(1, t.getVoteWrong());
		assertEquals(1, t.getConfirmedWrong());
		t.reset();
		assertEquals(0, t.getFrames());
		assertEquals(HiveState.AUDIENCE_CELL_UP, t.getTruth());
	}
}
