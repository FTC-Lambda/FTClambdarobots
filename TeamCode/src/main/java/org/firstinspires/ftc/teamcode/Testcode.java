package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Drive: Mecanum", group="LinearOpMode")
public class Testcode extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor backLeft  = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor topLeft   = hardwareMap.get(DcMotor.class, "top_left");
        DcMotor topRight  = hardwareMap.get(DcMotor.class, "top_right");
        

        waitForStart();

        while (opModeIsActive()) {
// ADJUST SENSITIVITY HERE
            // 1.0 = 100% speed, 0.5 = 50% speed, etc.
            double sensitivity = 0.5;

            // Get gamepad inputs (flipped for 180-degree robot orientation)
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Apply sensitivity and "Square" the inputs for smoother control at low speeds
            // (Math.signum preserves the forward/backward direction)
            y  = Math.signum(y)  * Math.pow(y, 2)  * sensitivity;
            x  = Math.signum(x)  * Math.pow(x, 2)  * sensitivity;
            rx = Math.signum(rx) * Math.pow(rx, 2) * sensitivity;

            // Mecanum calculations
            double frontLeft  = y + x + rx;
            double backLeftP  = y - x + rx;
            double frontRight = y - x - rx;
            double backRightP = y + x - rx;

            // Normalize powers so they don't exceed 1.0
            double max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeftP),
                    Math.max(Math.abs(frontRight), Math.abs(backRightP))));
            if (max > 1.0) {
                frontLeft /= max; backLeftP /= max; frontRight /= max; backRightP /= max;
            }

            topLeft.setPower(frontLeft);
            backLeft.setPower(backLeftP);
            topRight.setPower(frontRight);
            backRight.setPower(backRightP);

            telemetry.addData("Sensitivity", sensitivity);
            telemetry.addData("Left Stick Y", y);
            telemetry.update();
        }
    }
}