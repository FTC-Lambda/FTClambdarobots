package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp(name = "LambdaGoWild", group = "Linear OpMode")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(robot);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            telemetry.addData("Drive Input Y", gamepad1.left_stick_y);
            telemetry.addData("Drive Input X", gamepad1.left_stick_x);
            telemetry.addData("Sensitivity: ", Constants.DRIVE_SPEED);
            telemetry.addData("Team", "LAMBDA CHAMPIONS");
            telemetry.update();
        }
    }
}
