package org.firstinspires.ftc.teamcode.vision.hive;

import java.util.Locale;

/**
 * Scores the detector against a ground truth the tester enters, one count per camera frame.
 * Used by "Hive State Test" to measure how reliable detection is from each test spot.
 *
 * <p>Two scores are kept. The FRAME score judges each single-frame vote and shows how
 * readable the HIVE is from that spot. The CONFIRMED score judges the debounced output the
 * robot would act on. Changing the truth (after you TIP the HIVE) starts a new tally.</p>
 */
public final class HiveAccuracyTally {

	private HiveState truth = HiveState.UNKNOWN;
	private int frames;
	private int framesWithOwnTags;
	private int voteRight;
	private int voteWrong;
	private int noVote;
	private int confirmedRight;
	private int confirmedWrong;
	private int confirmedUnknown;

	/** Sets the real HIVE state and clears the counts. UNKNOWN pauses scoring. */
	public void setTruth(HiveState truth) {
		this.truth = truth == null ? HiveState.UNKNOWN : truth;
		reset();
	}

	public HiveState getTruth() {
		return truth;
	}

	/** Clears the counts and keeps the truth. */
	public void reset() {
		frames = 0;
		framesWithOwnTags = 0;
		voteRight = 0;
		voteWrong = 0;
		noVote = 0;
		confirmedRight = 0;
		confirmedWrong = 0;
		confirmedUnknown = 0;
	}

	/**
	 * Records one NEW camera frame.
	 *
	 * @param frameVote    the estimator's single-frame vote for this frame
	 * @param confirmed    the estimator's confirmed state after this frame
	 * @param ownTagsSeen  number of own-alliance HIVE tags used in this frame
	 */
	public void record(HiveState frameVote, HiveState confirmed, int ownTagsSeen) {
		if (truth == HiveState.UNKNOWN) {
			return;
		}
		frames++;
		if (ownTagsSeen > 0) {
			framesWithOwnTags++;
		}
		if (frameVote == null || frameVote == HiveState.UNKNOWN) {
			noVote++;
		} else if (frameVote == truth) {
			voteRight++;
		} else {
			voteWrong++;
		}
		if (confirmed == null || confirmed == HiveState.UNKNOWN) {
			confirmedUnknown++;
		} else if (confirmed == truth) {
			confirmedRight++;
		} else {
			confirmedWrong++;
		}
	}

	public int getFrames() { return frames; }
	public int getFramesWithOwnTags() { return framesWithOwnTags; }
	public int getVoteRight() { return voteRight; }
	public int getVoteWrong() { return voteWrong; }
	public int getNoVote() { return noVote; }
	public int getConfirmedRight() { return confirmedRight; }
	public int getConfirmedWrong() { return confirmedWrong; }
	public int getConfirmedUnknown() { return confirmedUnknown; }

	/** e.g. "right 92%  wrong 0%  no vote 8%  (seen 100% of 120 frames)" */
	public String frameSummary() {
		if (frames == 0) {
			return "no frames scored";
		}
		return String.format(Locale.US, "right %d%%  wrong %d%%  no vote %d%%  (seen %d%% of %d frames)",
				pct(voteRight), pct(voteWrong), pct(noVote), pct(framesWithOwnTags), frames);
	}

	/** e.g. "right 97%  wrong 0%  unknown 3%" */
	public String confirmedSummary() {
		if (frames == 0) {
			return "no frames scored";
		}
		return String.format(Locale.US, "right %d%%  wrong %d%%  unknown %d%%",
				pct(confirmedRight), pct(confirmedWrong), pct(confirmedUnknown));
	}

	private int pct(int n) {
		return (int) Math.round(100.0 * n / frames);
	}
}
