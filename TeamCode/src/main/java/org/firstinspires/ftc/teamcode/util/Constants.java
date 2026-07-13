package org.firstinspires.ftc.teamcode.util;

public final class Constants {
	private Constants() {}

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

	// --- Vision tracking PID: interactive follow (Limelight Seek Tag) ---
	public static final double VISION_SEEK_TURN_KP = 0.03;
	public static final double VISION_SEEK_TURN_KI = 0.002;
	public static final double VISION_SEEK_TURN_KD = 0.010;
	public static final double VISION_SEEK_TURN_INTEGRAL_LIMIT     = 50.0; // deg*s
	public static final double VISION_SEEK_TURN_INTEGRAL_ZONE_DEG = 10.0; // only integrate once roughly on-target

	public static final double VISION_SEEK_DRIVE_KP = 0.03;
	public static final double VISION_SEEK_DRIVE_KI = 0.004;
	public static final double VISION_SEEK_DRIVE_KD = 0.020;
	public static final double VISION_SEEK_DRIVE_INTEGRAL_LIMIT    = 50.0; // in*s
	public static final double VISION_SEEK_DRIVE_INTEGRAL_ZONE_IN  = 6.0;

	// --- Vision tracking PID: autonomous seek (AprilTag Seek Auto) ---
	public static final double VISION_AUTO_TURN_KP = 0.02;
	public static final double VISION_AUTO_TURN_KI = 0.0015;
	public static final double VISION_AUTO_TURN_KD = 0.008;
	public static final double VISION_AUTO_TURN_INTEGRAL_LIMIT     = 50.0;
	public static final double VISION_AUTO_TURN_INTEGRAL_ZONE_DEG  = 10.0;

	public static final double VISION_AUTO_DRIVE_KP = 0.02;
	public static final double VISION_AUTO_DRIVE_KI = 0.002;
	public static final double VISION_AUTO_DRIVE_KD = 0.015;
	public static final double VISION_AUTO_DRIVE_INTEGRAL_LIMIT    = 50.0;
	public static final double VISION_AUTO_DRIVE_INTEGRAL_ZONE_IN  = 6.0;

	// Low-pass filter on the PID derivative term (0..1: higher = snappier/less smoothing).
	// Raw frame-to-frame differentiation of a Limelight reading is noisy, so the derivative
	// (which also acts as feedforward "lead" on a moving tag) is filtered rather than used raw.
	public static final double VISION_TURN_DERIVATIVE_FILTER  = 0.35;
	public static final double VISION_DRIVE_DERIVATIVE_FILTER = 0.35;

	// Slew limits on the final commanded power (power units/sec), so corrections ramp instead
	// of snapping — protects the drivetrain and smooths out measurement-noise spikes.
	public static final double VISION_TURN_SLEW_RATE  = 3.0;
	public static final double VISION_DRIVE_SLEW_RATE = 2.5;
}
