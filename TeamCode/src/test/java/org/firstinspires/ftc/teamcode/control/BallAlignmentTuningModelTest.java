package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

public class BallAlignmentTuningModelTest {

	@Test
	public void selectionWrapsAndAdjustmentChangesOnlySelectedField() {
		BallAlignmentTuningModel m = new BallAlignmentTuningModel();
		assertEquals(BallAlignmentTuningModel.Parameter.TURN_KP, m.getSelectedParameter());
		double kdBefore = m.getAlignmentConfig().getTurnKd();
		assertEquals(BallAlignmentTuningModel.ChangeDomain.ALIGNMENT, m.adjust(1, false));
		assertEquals(Constants.BALL_ALIGN_TURN_KP + 0.002,
				m.getAlignmentConfig().getTurnKp(), 1e-9);
		assertEquals(kdBefore, m.getAlignmentConfig().getTurnKd(), 1e-9);
		m.selectPrevious();
		assertEquals(BallAlignmentTuningModel.Parameter.AIM_FILTER_GAIN,
				m.getSelectedParameter());
	}

	@Test
	public void coarseAdjustmentUsesDocumentedIncrement() {
		BallAlignmentTuningModel m = new BallAlignmentTuningModel();
		m.selectNext(); // TURN_KI
		m.selectNext(); // TURN_KD
		m.adjust(1, true);
		assertEquals(Constants.BALL_ALIGN_TURN_KD + 0.005,
				m.getAlignmentConfig().getTurnKd(), 1e-9);
	}

	@Test
	public void eachParameterAdjustmentOnlyReplacesItsDeclaredConfigDomain() {
		BallAlignmentTuningModel m = new BallAlignmentTuningModel();
		for (BallAlignmentTuningModel.Parameter parameter
				: BallAlignmentTuningModel.Parameter.values()) {
			assertEquals(parameter, m.getSelectedParameter());
			BallAlignmentConfig alignmentBefore = m.getAlignmentConfig();
			Object targetingBefore = m.getTargetingConfig();

			assertEquals(expectedDomain(parameter), m.adjust(1, false));
			if (expectedDomain(parameter) == BallAlignmentTuningModel.ChangeDomain.ALIGNMENT) {
				assertNotSame(alignmentBefore, m.getAlignmentConfig());
				assertSame(targetingBefore, m.getTargetingConfig());
			} else {
				assertSame(alignmentBefore, m.getAlignmentConfig());
				assertNotSame(targetingBefore, m.getTargetingConfig());
			}
			assertValid(m.getAlignmentConfig());
			assertValid(m.getTargetingConfig());
			m.selectNext();
		}
	}

	@Test
	public void directionNormalizesAndSelectedValuesExposeDefaultsAndIncrements() {
		BallAlignmentTuningModel m = new BallAlignmentTuningModel();
		assertEquals(Constants.BALL_ALIGN_TURN_KP, m.getSelectedValue(), 1e-9);
		assertEquals(Constants.BALL_ALIGN_TURN_KP, m.getSelectedDefaultValue(), 1e-9);
		assertEquals(0.002, m.getFineIncrement(), 1e-9);
		assertEquals(0.010, m.getCoarseIncrement(), 1e-9);
		m.adjust(50, false);
		assertEquals(Constants.BALL_ALIGN_TURN_KP + 0.002, m.getSelectedValue(), 1e-9);
		m.adjust(0, true);
		assertEquals(Constants.BALL_ALIGN_TURN_KP + 0.002, m.getSelectedValue(), 1e-9);
		m.adjust(-50, true);
		assertEquals(Constants.BALL_ALIGN_TURN_KP - 0.008, m.getSelectedValue(), 1e-9);
	}

	private static BallAlignmentTuningModel.ChangeDomain expectedDomain(
			BallAlignmentTuningModel.Parameter parameter) {
		switch (parameter) {
			case SWITCH_CONFIRM_FRAMES:
			case AIM_FILTER_GAIN:
				return BallAlignmentTuningModel.ChangeDomain.TARGETING;
			default:
				return BallAlignmentTuningModel.ChangeDomain.ALIGNMENT;
		}
	}

	private static void assertValid(BallAlignmentConfig config) {
		assertTrue(config.getTurnKp() >= 0.0);
		assertTrue(config.getTurnKi() >= 0.0);
		assertTrue(config.getTurnKd() >= 0.0);
		assertTrue(config.getDerivativeFilter() >= 0.0);
		assertTrue(config.getStopCorrectionDeg() <= config.getStartCorrectionDeg());
		assertTrue(config.getMaxTurnPower() <= 1.0);
		assertTrue(config.getStaticFrictionPower() <= config.getMaxTurnPower());
		assertTrue(config.getCommandHoldMs() >= 1L);
	}

	private static void assertValid(org.firstinspires.ftc.teamcode.vision.BallTargetingConfig config) {
		assertTrue(config.getSwitchConfirmFrames() >= 1);
		assertTrue(config.getSwitchConfirmFrames() <= 10);
		assertTrue(config.getAimFilterGain() > 0.0);
		assertTrue(config.getAimFilterGain() <= 1.0);
	}
}
