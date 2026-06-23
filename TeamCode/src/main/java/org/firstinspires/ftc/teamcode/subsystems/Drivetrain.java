package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Drivetrain {

	private final RobotHardware robot;

	public Drivetrain(RobotHardware robot) {
		this.robot = robot;
	}

	public void drive(double y, double x, double rx) {
		//Control orientation
		double adjustedY = -y;
    //Strafing
		double adjustedX = -x;
    //Rotation
		double adjustedRx = rx;

		//square input behaviour so it travels more precisely with small joystick movements. 
		adjustedY = Math.signum(adjustedY) * Math.pow(adjustedY, 2) * Constants.DRIVE_SPEED;
		adjustedX = Math.signum(adjustedX) * Math.pow(adjustedX, 2) * Constants.DRIVE_SPEED;
		adjustedRx = Math.signum(adjustedRx) * Math.pow(adjustedRx, 2) * Constants.DRIVE_SPEED;

//Drivetrain equation that incorporates both strafing and rotations into the wheels
		double frontLeft = adjustedY + adjustedX + adjustedRx;
		double backLeftP = adjustedY - adjustedX + adjustedRx;
		double frontRight = adjustedY - adjustedX - adjustedRx;
		double backRightP = adjustedY + adjustedX - adjustedRx;

		double max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeftP),
				Math.max(Math.abs(frontRight), Math.abs(backRightP))));
		if (max > 1.0) {
			frontLeft /= max;
			backLeftP /= max;
			frontRight /= max;
			backRightP /= max;
		}

		robot.topLeft.setPower(frontLeft);
		robot.backLeft.setPower(backLeftP);
		robot.topRight.setPower(frontRight);
		robot.backRight.setPower(backRightP);
	}