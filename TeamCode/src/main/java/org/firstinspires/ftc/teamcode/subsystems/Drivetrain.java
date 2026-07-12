package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Drivetrain {

	private final RobotHardware robot;

	// Per-motor scale factors for calibration (index: 0=topLeft, 1=backLeft, 2=topRight, 3=backRight)
	private double[] motorScales = {
			Constants.TOP_LEFT_SCALE,
			Constants.BACK_LEFT_SCALE,
			Constants.TOP_RIGHT_SCALE,
			Constants.BACK_RIGHT_SCALE
	};

	public static final int MOTOR_TOP_LEFT  = 0;
	public static final int MOTOR_BACK_LEFT  = 1;
	public static final int MOTOR_TOP_RIGHT  = 2;
	public static final int MOTOR_BACK_RIGHT = 3;

	public Drivetrain(RobotHardware robot) {
		this.robot = robot;
	}

	public void setMotorScale(int motorIndex, double scale) {
		if (motorIndex >= 0 && motorIndex < motorScales.length) {
			motorScales[motorIndex] = Math.max(Constants.MOTOR_SCALE_MIN, Math.min(Constants.MOTOR_SCALE_MAX, scale));
		}
	}

	public double getMotorScale(int motorIndex) {
		if (motorIndex >= 0 && motorIndex < motorScales.length) {
			return motorScales[motorIndex];
		}
		return 1.0;
	}

	public void drive(double y, double x, double rx) {
		//Control orientation
		double adjustedY = -y;
		//Strafing
		double adjustedX = x;
		//Rotation
		double adjustedRx = rx;

		//square input behaviour so it travels more precisely with small joystick movements.
		adjustedY = Math.signum(adjustedY) * Math.pow(adjustedY, 2) * Constants.DRIVE_SPEED;
		adjustedX = Math.signum(adjustedX) * Math.pow(adjustedX, 2) * Constants.DRIVE_SPEED;
		adjustedRx = Math.signum(adjustedRx) * Math.pow(adjustedRx, 2) * Constants.DRIVE_SPEED;

		driveMecanum(adjustedY, adjustedX, adjustedRx);
	}

	public void driveRaw(double y, double x, double rx) {
		driveMecanum(y, x, rx);
	}

	private void driveMecanum(double y, double x, double rx) {
		//Drivetrain equation that incorporates both strafing and rotations into the wheels
		double frontLeft  =  y + x + rx;
		double backLeft   =  y - x + rx;
		double frontRight =  y - x - rx;
		double backRight  =  y + x - rx;

		// Normalize against raw (unscaled) values so the calibration scales are not
		// counteracted by the normalization denominator.
		double max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeft),
				Math.max(Math.abs(frontRight), Math.abs(backRight))));
		if (max > 1.0) {
			frontLeft  /= max;
			backLeft   /= max;
			frontRight /= max;
			backRight  /= max;
		}

		// Apply per-motor calibration scales after normalization, clamping to [-1, 1].
		robot.topLeft.setPower(clampPower(frontLeft  * motorScales[MOTOR_TOP_LEFT]));
		robot.backLeft.setPower(clampPower(backLeft   * motorScales[MOTOR_BACK_LEFT]));
		robot.topRight.setPower(clampPower(frontRight * motorScales[MOTOR_TOP_RIGHT]));
		robot.backRight.setPower(clampPower(backRight  * motorScales[MOTOR_BACK_RIGHT]));
	}

	private static double clampPower(double power) {
		return Math.max(-1.0, Math.min(1.0, power));
	}
}
