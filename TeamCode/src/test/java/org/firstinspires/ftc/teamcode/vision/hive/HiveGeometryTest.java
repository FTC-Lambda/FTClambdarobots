package org.firstinspires.ftc.teamcode.vision.hive;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class HiveGeometryTest {
	private static final double EPS = 1e-9;
	private static final double CAM_H = 10.0;

	@Test
	public void rangeIsEuclideanNorm() {
		assertEquals(13.0, HiveGeometry.rangeInches(3, 4, 12), EPS);
	}

	@Test
	public void rangeIsNaNForAllZero() {
		assertTrue(Double.isNaN(HiveGeometry.rangeInches(0, 0, 0)));
	}

	@Test
	public void rangeIsNaNForNonFiniteInput() {
		assertTrue(Double.isNaN(HiveGeometry.rangeInches(Double.NaN, 4, 12)));
		assertTrue(Double.isNaN(HiveGeometry.rangeInches(3, Double.POSITIVE_INFINITY, 12)));
		assertTrue(Double.isNaN(HiveGeometry.rangeInches(3, 4, Double.NaN)));
	}

	@Test
	public void levelCameraOnAxisSeesTagAtCameraHeight() {
		assertEquals(CAM_H, HiveGeometry.tagHeightInches(20, 0, 0, CAM_H, 0), EPS);
	}

	@Test
	public void tyAddsRangeTimesSinOfTy() {
		// ty = 30 deg, tx = 0: up component is sin(30) = 0.5, so 20 * 0.5 = 10 in higher
		assertEquals(CAM_H + 10.0, HiveGeometry.tagHeightInches(20, 0, 30, CAM_H, 0), EPS);
	}

	@Test
	public void pitchAddsRangeTimesSinOfPitch() {
		assertEquals(CAM_H + 10.0, HiveGeometry.tagHeightInches(20, 0, 0, CAM_H, 30), EPS);
	}

	@Test
	public void symmetricInTxSign() {
		double left = HiveGeometry.tagHeightInches(30, -20, 15, CAM_H, 25);
		double right = HiveGeometry.tagHeightInches(30, 20, 15, CAM_H, 25);
		assertEquals(left, right, EPS);
	}

	@Test
	public void matchesIndependentRotationOfCameraRay() {
		double range = 30, txDeg = 20, tyDeg = 15, pitchDeg = 25;

		// Camera-frame unit ray: x right, y up, z forward.
		double x = Math.tan(Math.toRadians(txDeg));
		double y = Math.tan(Math.toRadians(tyDeg));
		double z = 1.0;
		double len = Math.sqrt(x * x + y * y + z * z);
		x /= len;
		y /= len;
		z /= len;
		// Rotate about the camera x axis by the pitch (nose up): world up = y*cos + z*sin.
		double p = Math.toRadians(pitchDeg);
		double worldUp = Math.cos(p) * y + Math.sin(p) * z;
		double expected = CAM_H + range * worldUp;

		double actual = HiveGeometry.tagHeightInches(range, txDeg, tyDeg, CAM_H, pitchDeg);
		assertEquals(expected, actual, EPS);
		// Value computed separately (Python) for the same inputs.
		assertEquals(28.19210026740921, actual, 1e-6);
	}

	@Test
	public void invalidInputsGiveNaN() {
		assertTrue(Double.isNaN(HiveGeometry.tagHeightInches(0, 0, 0, CAM_H, 0)));
		assertTrue(Double.isNaN(HiveGeometry.tagHeightInches(-5, 0, 0, CAM_H, 0)));
		assertTrue(Double.isNaN(HiveGeometry.tagHeightInches(Double.NaN, 0, 0, CAM_H, 0)));
		assertTrue(Double.isNaN(HiveGeometry.tagHeightInches(20, Double.NaN, 0, CAM_H, 0)));
		assertTrue(Double.isNaN(HiveGeometry.tagHeightInches(20, 0, Double.NaN, CAM_H, 0)));
		assertTrue(Double.isNaN(HiveGeometry.tagHeightInches(20, 0, 0, Double.NaN, 0)));
		assertTrue(Double.isNaN(HiveGeometry.tagHeightInches(20, 0, 0, CAM_H, Double.NaN)));
		assertTrue(Double.isNaN(HiveGeometry.tagHeightInches(Double.POSITIVE_INFINITY, 0, 0, CAM_H, 0)));
	}
}
