package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * End-to-end check of the pure HIVE pipeline, the same chain HiveDetector runs:
 * synthetic Limelight readings (tx, ty, range) -> HiveGeometry height -> HiveTagMap ->
 * HiveTagObservation -> HiveStateEstimator.
 */
public class HivePipelineIntegrationTest {

	private static final double CAM_H = 10.0;
	private static final double CAM_PITCH = 20.0;

	/** What the Limelight would report for a tag at a known world position relative to the camera. */
	private static double[] simulate(double rightIn, double tagHeightIn, double forwardIn) {
		double up = tagHeightIn - CAM_H;
		double p = Math.toRadians(CAM_PITCH);
		// Rotate the world vector into the camera frame (camera pitched up by p; x right, y up, z fwd).
		double cx = rightIn;
		double cy = up * Math.cos(p) - forwardIn * Math.sin(p);
		double cz = up * Math.sin(p) + forwardIn * Math.cos(p);
		double tx = Math.toDegrees(Math.atan(cx / cz));
		double ty = Math.toDegrees(Math.atan(cy / cz));
		double range = HiveGeometry.rangeInches(cx, cy, cz);
		return new double[] {tx, ty, range};
	}

	private static HiveTagObservation observe(int id, double rightIn, double tagHeightIn, double forwardIn) {
		double[] ll = simulate(rightIn, tagHeightIn, forwardIn);
		double h = HiveGeometry.tagHeightInches(ll[2], ll[0], ll[1], CAM_H, CAM_PITCH);
		return new HiveTagObservation(id, HiveTagMap.allianceOf(id), HiveTagMap.cellOf(id), h);
	}

	/** A 4-tag cluster on one CELL, tags spread horizontally, all at roughly the same height. */
	private static List<HiveTagObservation> cluster(Alliance alliance, HiveCell cell,
			double heightIn, double forwardIn) {
		List<HiveTagObservation> obs = new ArrayList<>();
		int[] ids = HiveTagMap.tagIds(alliance, cell);
		for (int i = 0; i < ids.length; i++) {
			obs.add(observe(ids[i], -7.5 + 5.0 * i, heightIn + 0.3 * i, forwardIn));
		}
		return obs;
	}

	@Test
	public void geometryRecoversTrueTagHeightFromSimulatedLimelightReadings() {
		double[][] cases = {{0, 45, 60}, {12, 33, 40}, {-20, 50, 90}, {5, 31, 24}};
		for (double[] c : cases) {
			double[] ll = simulate(c[0], c[1], c[2]);
			double h = HiveGeometry.tagHeightInches(ll[2], ll[0], ll[1], CAM_H, CAM_PITCH);
			assertEquals(c[1], h, 1e-6);
		}
	}

	@Test
	public void raisedFarCellOfOurHiveConfirmsFarCellUp() {
		HiveStateEstimator est = new HiveStateEstimator(Alliance.RED, HiveDetectionConfig.defaults());
		for (int f = 0; f < 3; f++) {
			est.update(1000L + 50L * f, cluster(Alliance.RED, HiveCell.FAR, 47.0, 70.0));
		}
		assertEquals(HiveState.FAR_CELL_UP, est.getState());
		assertEquals(CellPosition.RAISED, est.getCellPosition(HiveCell.FAR));
		assertEquals(4, est.getCellTagCount(HiveCell.FAR));
	}

	@Test
	public void frameVoteIsImmediateWhileConfirmedStateWaits() {
		HiveStateEstimator est = new HiveStateEstimator(Alliance.RED, HiveDetectionConfig.defaults());
		est.update(1000L, cluster(Alliance.RED, HiveCell.FAR, 47.0, 70.0));
		assertEquals(HiveState.FAR_CELL_UP, est.getLastFrameVote());
		assertEquals(HiveState.UNKNOWN, est.getState());
		est.update(1050L, new ArrayList<HiveTagObservation>());
		assertEquals(HiveState.UNKNOWN, est.getLastFrameVote());
	}

	@Test
	public void loweredAudienceCellSeenFromAudienceSideAlsoMeansFarCellUp() {
		HiveStateEstimator est = new HiveStateEstimator(Alliance.BLUE, HiveDetectionConfig.defaults());
		for (int f = 0; f < 3; f++) {
			est.update(1000L + 50L * f, cluster(Alliance.BLUE, HiveCell.AUDIENCE, 33.0, 50.0));
		}
		assertEquals(HiveState.FAR_CELL_UP, est.getState());
		assertEquals(CellPosition.LOWERED, est.getCellPosition(HiveCell.AUDIENCE));
	}

	@Test
	public void opponentHiveTagsAreIgnoredAndTipIsTrackedAfterConfirmation() {
		HiveStateEstimator est = new HiveStateEstimator(Alliance.RED, HiveDetectionConfig.defaults());
		long t = 1000L;
		// Opponent (blue) far CELL raised in view: must not affect RED's state.
		for (int f = 0; f < 5; f++) {
			est.update(t += 50, cluster(Alliance.BLUE, HiveCell.FAR, 48.0, 60.0));
		}
		assertEquals(HiveState.UNKNOWN, est.getState());

		// Our audience CELL raised -> AUDIENCE_CELL_UP.
		for (int f = 0; f < 3; f++) {
			est.update(t += 50, cluster(Alliance.RED, HiveCell.AUDIENCE, 46.0, 55.0));
		}
		assertEquals(HiveState.AUDIENCE_CELL_UP, est.getState());

		// HIVE TIPS: audience CELL now lowered. Two frames are not enough...
		est.update(t += 50, cluster(Alliance.RED, HiveCell.AUDIENCE, 32.0, 55.0));
		est.update(t += 50, cluster(Alliance.RED, HiveCell.AUDIENCE, 32.0, 55.0));
		assertEquals(HiveState.AUDIENCE_CELL_UP, est.getState());
		// ...the third confirms the TIP.
		est.update(t += 50, cluster(Alliance.RED, HiveCell.AUDIENCE, 32.0, 55.0));
		assertEquals(HiveState.FAR_CELL_UP, est.getState());
		assertTrue(est.isFresh(t));
	}
}
