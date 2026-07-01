package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {

	public DcMotor backLeft;
	public DcMotor backRight;
	public DcMotor topLeft;
	public DcMotor topRight;

	public Limelight3A limelight;

	public void init(HardwareMap hardwareMap) {
		backLeft = hardwareMap.get(DcMotor.class, "back_left");
		backRight = hardwareMap.get(DcMotor.class, "back_right");
		topLeft = hardwareMap.get(DcMotor.class, "top_left");
		topRight = hardwareMap.get(DcMotor.class, "top_right");

		limelight = hardwareMap.get(Limelight3A.class, "limelight");
	}
}
