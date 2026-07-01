package org.firstinspires.ftc.teamcode.util;

public final class Constants {
	// Adjust sensitivity of the wheels
	public static final double DRIVE_SPEED = 0.5;

	// Motor calibration scale bounds
	public static final double MOTOR_SCALE_MIN  = 0.0;
	public static final double MOTOR_SCALE_MAX  = 2.0;
	public static final double MOTOR_SCALE_STEP = 0.05;

	// Per-motor power calibration (field-tuned defaults)
	public static final double TOP_LEFT_SCALE  = 1.00;
	public static final double BACK_LEFT_SCALE = 1.40;
	public static final double TOP_RIGHT_SCALE = 1.00;
	public static final double BACK_RIGHT_SCALE = 1.40;

	// Vision follow / approach deadbands (tune on Driver Station in Limelight Seek Tag)
	public static final double VISION_BEARING_DEADBAND_DEG = 5.0;
	public static final double VISION_DISTANCE_DEADBAND_IN  = 3.0;
	public static final double VISION_BEARING_DEADBAND_STEP = 0.5;
	public static final double VISION_DISTANCE_DEADBAND_STEP = 0.5;
	public static final double VISION_BEARING_DEADBAND_MIN  = 1.0;
	public static final double VISION_BEARING_DEADBAND_MAX  = 15.0;
	public static final double VISION_DISTANCE_DEADBAND_MIN   = 0.5;
	public static final double VISION_DISTANCE_DEADBAND_MAX   = 12.0;
	// Hysteresis: start correcting above deadband+start, stop below deadband-stop
	public static final double VISION_BEARING_HYSTERESIS_START_OFFSET  = 1.0;
	public static final double VISION_BEARING_HYSTERESIS_STOP_OFFSET   = 2.0;
	public static final double VISION_DISTANCE_HYSTERESIS_START_OFFSET = 0.5;
	public static final double VISION_DISTANCE_HYSTERESIS_STOP_OFFSET  = 1.0;
}
