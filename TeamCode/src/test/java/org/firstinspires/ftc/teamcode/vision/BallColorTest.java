package org.firstinspires.ftc.teamcode.vision;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class BallColorTest {

	@Test
	public void parsesClassNames() {
		assertEquals(BallColor.GREEN, BallColor.fromClassName("green"));
		assertEquals(BallColor.GREEN, BallColor.fromClassName("GreenBall"));
		assertEquals(BallColor.PURPLE, BallColor.fromClassName("purple"));
		assertEquals(BallColor.UNKNOWN, BallColor.fromClassName("orange"));
		assertEquals(BallColor.UNKNOWN, BallColor.fromClassName(null));
	}
}
