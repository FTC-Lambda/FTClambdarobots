package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {

	public DcMotor backLeft;
	public DcMotor backRight;
	public DcMotor topLeft;
	public DcMotor topRight;

	public void init(HardwareMap hardwareMap) {
		backLeft = hardwareMap.get(DcMotor.class, "back_left");
		backRight = hardwareMap.get(DcMotor.class, "back_right");
		topLeft = hardwareMap.get(DcMotor.class, "top_left");
		topRight = hardwareMap.get(DcMotor.class, "top_right");
	}
}
