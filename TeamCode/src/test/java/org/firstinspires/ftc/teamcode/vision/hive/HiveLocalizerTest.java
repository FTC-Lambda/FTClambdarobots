package org.firstinspires.ftc.teamcode.vision.hive;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class HiveLocalizerTest {

	private static final double EPS = 1e-6;
	private static final CameraMount MOUNT = new CameraMount(6.0, -2.0, 10.0, 5.0, 25.0);
	private static final HiveFieldModel MODEL = HiveFieldModel.defaults();

	private static HiveLocalizer localizer() {
		return new HiveLocalizer(MODEL, MOUNT, HiveDetectionConfig.defaults(), 6.0, 4.0);
	}

	/** Inverse of the pipeline: what the Limelight reports for a tag at a known field position. */
	private static TagSighting see(int id, Vec3 tag, double rx, double ry, double headingDeg, CameraMount m) {
		double h = Math.toRadians(headingDeg);
		double dx = tag.x - rx;
		double dy = tag.y - ry;
		double bx = Math.cos(h) * dx + Math.sin(h) * dy;     // field -> robot frame
		double by = -Math.sin(h) * dx + Math.cos(h) * dy;
		double vx = bx - m.forwardIn;
		double vy = by - m.leftIn;
		double vz = tag.z - m.heightIn;
		double yaw = Math.toRadians(m.yawDeg);
		double fwdLevel = Math.cos(yaw) * vx + Math.sin(yaw) * vy;
		double left = -Math.sin(yaw) * vx + Math.cos(yaw) * vy;
		double p = Math.toRadians(m.pitchDeg);
		double fwd = fwdLevel * Math.cos(p) + vz * Math.sin(p);
		double up = -fwdLevel * Math.sin(p) + vz * Math.cos(p);
		double right = -left;
		double range = Math.sqrt(fwd * fwd + up * up + right * right);
		return new TagSighting(id, range, Math.toDegrees(Math.atan(right / fwd)),
				Math.toDegrees(Math.atan(up / fwd)));
	}

	private static List<TagSighting> seeCluster(Alliance a, HiveCell c, CellPosition pos,
			double rx, double ry, double hDeg) {
		List<TagSighting> out = new ArrayList<>();
		for (int id : HiveTagMap.tagIds(a, c)) {
			out.add(see(id, MODEL.tagPosition(id, pos), rx, ry, hDeg, MOUNT));
		}
		return out;
	}

	// ---- CameraMount -------------------------------------------------------------------

	@Test
	public void mountHeightMatchesHiveGeometry() {
		Vec3 p = MOUNT.toRobotFrame(50.0, 7.0, 12.0);
		assertEquals(HiveGeometry.tagHeightInches(50.0, 7.0, 12.0, 10.0, 25.0), p.z, 1e-9);
	}

	@Test
	public void mountYawAndOffsets() {
		CameraMount m = new CameraMount(3.0, 1.0, 8.0, 90.0, 0.0);
		Vec3 p = m.toRobotFrame(20.0, 0.0, 0.0);      // straight out of a camera aimed left
		assertEquals(3.0, p.x, 1e-9);
		assertEquals(21.0, p.y, 1e-9);
		assertEquals(8.0, p.z, 1e-9);
		CameraMount level = new CameraMount(0, 0, 0, 0, 0);
		Vec3 r = level.toRobotFrame(10.0, 45.0, 0.0);   // 45 deg to the camera's right
		assertEquals(10.0 / Math.sqrt(2), r.x, 1e-9);
		assertEquals(-10.0 / Math.sqrt(2), r.y, 1e-9);
		assertNull(level.toRobotFrame(Double.NaN, 0, 0));
		assertNull(level.toRobotFrame(0.0, 0, 0));
	}

	// ---- HiveFieldModel ------------------------------------------------------------------

	@Test
	public void modelHeightsAndPositions() {
		Vec3 raisedFar = MODEL.clusterCenter(Alliance.RED, HiveCell.FAR, CellPosition.RAISED);
		Vec3 loweredAud = MODEL.clusterCenter(Alliance.RED, HiveCell.AUDIENCE, CellPosition.LOWERED);
		assertEquals(49.05, raisedFar.z, 0.05);
		assertEquals(33.61, loweredAud.z, 0.05);
		assertEquals(72.0 + 14.89, raisedFar.y, 0.05);
		assertEquals(72.0 - 11.85, loweredAud.y, 0.05);
		assertEquals(59.25, raisedFar.x, 1e-9);
		assertEquals(84.75, MODEL.clusterCenter(Alliance.BLUE, HiveCell.FAR, CellPosition.RAISED).x, 1e-9);
		// Mirror symmetry between the two ends.
		Vec3 raisedAud = MODEL.clusterCenter(Alliance.RED, HiveCell.AUDIENCE, CellPosition.RAISED);
		assertEquals(raisedFar.z, raisedAud.z, 1e-9);
		assertEquals(144.0 - raisedFar.y, raisedAud.y, 1e-9);
		assertNull(MODEL.clusterCenter(Alliance.RED, HiveCell.FAR, CellPosition.AMBIGUOUS));
	}

	@Test
	public void modelReproducesManualLowPoint() {
		// Lowered CELL's outer bottom edge: 30.6 in above the tiles (Figure 9-10), using the
		// face's outer edge 12.04/2 in beyond the face center.
		HiveFieldModel.Params d = HiveFieldModel.Params.defaults();
		HiveFieldModel edge = new HiveFieldModel(new HiveFieldModel.Params(d.fieldCenterXIn, d.fieldCenterYIn,
				d.hiveSpacingIn, d.pivotHeightIn, d.armHalfLengthIn, d.pivotAboveArmIn, d.tiltDeg,
				d.tagOffsetsIn, 6.02));
		assertEquals(30.6, edge.clusterCenter(Alliance.RED, HiveCell.AUDIENCE, CellPosition.LOWERED).z, 0.1);
	}

	@Test
	public void tagOrderMatchesFigure917() {
		// Audience clusters: ascending IDs run toward +x. Far clusters: toward -x.
		assertTrue(MODEL.tagPosition(34, CellPosition.RAISED).x < MODEL.tagPosition(37, CellPosition.RAISED).x);
		assertTrue(MODEL.tagPosition(38, CellPosition.RAISED).x < MODEL.tagPosition(41, CellPosition.RAISED).x);
		assertTrue(MODEL.tagPosition(30, CellPosition.RAISED).x > MODEL.tagPosition(33, CellPosition.RAISED).x);
		assertTrue(MODEL.tagPosition(42, CellPosition.RAISED).x > MODEL.tagPosition(45, CellPosition.RAISED).x);
		assertEquals(59.25 - 6.5, MODEL.tagPosition(34, CellPosition.LOWERED).x, 1e-9);
		assertEquals(59.25 + 2.75, MODEL.tagPosition(31, CellPosition.LOWERED).x, 1e-9);
		assertNull(MODEL.tagPosition(20, CellPosition.RAISED));
	}

	@Test
	public void positionForState() {
		assertEquals(CellPosition.RAISED, HiveFieldModel.positionFor(HiveCell.FAR, HiveState.FAR_CELL_UP));
		assertEquals(CellPosition.LOWERED, HiveFieldModel.positionFor(HiveCell.AUDIENCE, HiveState.FAR_CELL_UP));
		assertNull(HiveFieldModel.positionFor(HiveCell.FAR, HiveState.UNKNOWN));
	}

	// ---- HiveLocalizer --------------------------------------------------------------------

	@Test
	public void headingGivenRecoversPoseFromOneCluster() {
		double x = 40.0, y = 20.0, h = 62.6;
		FieldPoseEstimate e = localizer().estimate(
				seeCluster(Alliance.RED, HiveCell.AUDIENCE, CellPosition.RAISED, x, y, h), h);
		assertTrue(e.valid);
		assertEquals(FieldPoseEstimate.Mode.HEADING_GIVEN, e.mode);
		assertEquals(x, e.xIn, EPS);
		assertEquals(y, e.yIn, EPS);
		assertEquals(h, e.headingDeg, EPS);
		assertEquals(4, e.tagsUsed);
		assertEquals(0.0, e.rmsResidualIn, EPS);
		assertEquals(0.0, e.rmsHeightErrorIn, EPS);
		assertTrue(e.clusters.contains("R-A RAISED"));
	}

	@Test
	public void headingGivenWorksWithASingleTag() {
		double x = 30.0, y = 110.0, h = -70.0;
		List<TagSighting> one = new ArrayList<>();
		one.add(see(31, MODEL.tagPosition(31, CellPosition.RAISED), x, y, h, MOUNT));
		FieldPoseEstimate e = localizer().estimate(one, h);
		assertTrue(e.valid);
		assertEquals(x, e.xIn, EPS);
		assertEquals(y, e.yIn, EPS);
		assertFalse(localizer().estimate(one, Double.NaN).valid);   // no heading: needs 2+ tags
	}

	@Test
	public void tagsOnlyFitsHeadingFromOneCluster() {
		double x = 70.0, y = 125.0, h = -95.0;
		FieldPoseEstimate e = localizer().estimate(
				seeCluster(Alliance.BLUE, HiveCell.FAR, CellPosition.RAISED, x, y, h), Double.NaN);
		assertTrue(e.valid);
		assertEquals(FieldPoseEstimate.Mode.TAGS_ONLY, e.mode);
		assertEquals(x, e.xIn, EPS);
		assertEquals(y, e.yIn, EPS);
		assertEquals(h, e.headingDeg, EPS);
	}

	@Test
	public void tagsOnlyUsesBothClustersAndBothHives() {
		// From the audience end with AUDIENCE UP: near raised + far lowered of both HIVEs.
		double x = 75.0, y = 15.0, h = 88.0;
		List<TagSighting> s = new ArrayList<>();
		s.addAll(seeCluster(Alliance.RED, HiveCell.AUDIENCE, CellPosition.RAISED, x, y, h));
		s.addAll(seeCluster(Alliance.RED, HiveCell.FAR, CellPosition.LOWERED, x, y, h));
		s.addAll(seeCluster(Alliance.BLUE, HiveCell.AUDIENCE, CellPosition.RAISED, x, y, h));
		FieldPoseEstimate e = localizer().estimate(s, Double.NaN);
		assertTrue(e.valid);
		assertEquals(12, e.tagsUsed);
		assertEquals(x, e.xIn, EPS);
		assertEquals(y, e.yIn, EPS);
		assertEquals(h, e.headingDeg, EPS);
		assertTrue(e.clusters.contains("R-F LOWERED"));
		assertTrue(e.clusters.contains("B-A RAISED"));
	}

	@Test
	public void rejectsAnOutlierTag() {
		double x = 40.0, y = 20.0, h = 62.6;
		List<TagSighting> s = seeCluster(Alliance.RED, HiveCell.AUDIENCE, CellPosition.RAISED, x, y, h);
		TagSighting bad = s.get(1);
		s.set(1, new TagSighting(bad.id, bad.rangeIn, bad.txDeg + 12.0, bad.tyDeg));   // wrong bearing
		FieldPoseEstimate e = localizer().estimate(s, h);
		assertTrue(e.valid);
		assertEquals(3, e.tagsUsed);
		assertEquals(1, e.tagsRejected);
		assertEquals(x, e.xIn, EPS);
		assertEquals(y, e.yIn, EPS);
	}

	@Test
	public void ambiguousClusterIsNotUsed() {
		// A cluster measured right at the threshold (40 in) cannot be placed.
		Vec3 mid = new Vec3(59.25, 60.0, 40.0);
		List<TagSighting> s = new ArrayList<>();
		s.add(see(34, mid, 50.0, 20.0, 80.0, MOUNT));
		s.add(see(35, new Vec3(62.0, 60.0, 40.0), 50.0, 20.0, 80.0, MOUNT));
		FieldPoseEstimate e = localizer().estimate(s, 80.0);
		assertFalse(e.valid);
		assertEquals("clusters ambiguous", e.reason);
		assertTrue(e.clusters.contains("R-A AMBIGUOUS"));
	}

	@Test
	public void noHiveTags() {
		List<TagSighting> s = new ArrayList<>();
		s.add(new TagSighting(20, 50.0, 0.0, 10.0));
		FieldPoseEstimate e = localizer().estimate(s, Double.NaN);
		assertFalse(e.valid);
		assertEquals("no HIVE tags", e.reason);
		assertNotNull(e.summary());
	}

	@Test
	public void wrongMountShowsUpAsHeightError() {
		double x = 40.0, y = 20.0, h = 62.6;
		List<TagSighting> s = seeCluster(Alliance.RED, HiveCell.AUDIENCE, CellPosition.RAISED, x, y, h);
		HiveLocalizer off = new HiveLocalizer(MODEL, MOUNT.withHeightAndPitch(12.0, 25.0),
				HiveDetectionConfig.defaults(), 6.0, 4.0);
		FieldPoseEstimate e = off.estimate(s, h);
		assertTrue(e.valid);
		assertEquals(2.0, e.rmsHeightErrorIn, 1e-6);
	}

	@Test
	public void tileNames() {
		assertEquals("A1", HiveLabels.tileName(0.0, 0.0));
		assertEquals("B1", HiveLabels.tileName(30.0, 10.0));
		assertEquals("F6", HiveLabels.tileName(144.0, 144.0));
		assertEquals("C4", HiveLabels.tileName(59.25, 86.9));
		assertEquals("off field", HiveLabels.tileName(-1.0, 50.0));
		assertEquals("off field", HiveLabels.tileName(Double.NaN, 50.0));
	}

	@Test
	public void normalizesHeading() {
		assertEquals(-170.0, HiveLocalizer.normalizeDeg(190.0), 1e-9);
		assertEquals(180.0, HiveLocalizer.normalizeDeg(-180.0), 1e-9);
		assertEquals(10.0, HiveLocalizer.normalizeDeg(370.0), 1e-9);
	}
}
