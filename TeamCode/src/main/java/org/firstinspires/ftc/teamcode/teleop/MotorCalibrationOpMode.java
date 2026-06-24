package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;

/**
 * Motor Calibration OpMode
 *
 * Controls:
 *   Gamepad 1 left stick  — drive (strafe + forward/back)
 *   Gamepad 1 right stick — rotate
 *
 *   D-pad UP    — select Top-Left motor
 *   D-pad LEFT  — select Back-Left motor
 *   D-pad RIGHT — select Top-Right motor
 *   D-pad DOWN  — select Back-Right motor
 *
 *   Right bumper (RB) — increase selected motor scale by MOTOR_SCALE_STEP
 *   Left bumper  (LB) — decrease selected motor scale by MOTOR_SCALE_STEP
 *   Y button          — reset all scales to 1.0
 */
@TeleOp(name = "Motor Calibration", group = "Calibration")
public class MotorCalibrationOpMode extends LinearOpMode {

    private static final String[] MOTOR_NAMES = {"Top-Left", "Back-Left", "Top-Right", "Back-Right"};

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        Drivetrain drivetrain = new Drivetrain(robot);

        int selectedMotor = Drivetrain.MOTOR_TOP_LEFT;

        // Button edge-detection state
        boolean prevRB = false, prevLB = false, prevY = false;
        boolean prevDpadUp = false, prevDpadLeft = false, prevDpadRight = false, prevDpadDown = false;

        telemetry.addLine("Motor Calibration ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // --- Motor selection via d-pad (edge-triggered) ---
            if (gamepad1.dpad_up    && !prevDpadUp)   selectedMotor = Drivetrain.MOTOR_TOP_LEFT;
            if (gamepad1.dpad_left  && !prevDpadLeft)  selectedMotor = Drivetrain.MOTOR_BACK_LEFT;
            if (gamepad1.dpad_right && !prevDpadRight) selectedMotor = Drivetrain.MOTOR_TOP_RIGHT;
            if (gamepad1.dpad_down  && !prevDpadDown)  selectedMotor = Drivetrain.MOTOR_BACK_RIGHT;

            prevDpadUp    = gamepad1.dpad_up;
            prevDpadLeft  = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;
            prevDpadDown  = gamepad1.dpad_down;

            // --- Scale adjustment via bumpers (edge-triggered, mutually exclusive) ---
            // else-if prevents simultaneous press from writing two conflicting values
            // with a stale currentScale, which would silently discard the RB increment.
            double currentScale = drivetrain.getMotorScale(selectedMotor);
            if (gamepad1.right_bumper && !prevRB) {
                drivetrain.setMotorScale(selectedMotor, currentScale + Constants.MOTOR_SCALE_STEP);
            } else if (gamepad1.left_bumper && !prevLB) {
                drivetrain.setMotorScale(selectedMotor, currentScale - Constants.MOTOR_SCALE_STEP);
            }

            prevRB = gamepad1.right_bumper;
            prevLB = gamepad1.left_bumper;

            // --- Reset all scales (Y button, edge-triggered) ---
            if (gamepad1.y && !prevY) {
                for (int i = 0; i < MOTOR_NAMES.length; i++) drivetrain.setMotorScale(i, 1.0);
            }
            prevY = gamepad1.y;

            // --- Drive ---
            drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // --- Telemetry ---
            telemetry.addLine("=== Motor Calibration ===");
            telemetry.addLine("D-pad: select motor | LB/RB: adjust scale | Y: reset all");
            telemetry.addLine("");
            for (int i = 0; i < MOTOR_NAMES.length; i++) {
                String prefix = (i == selectedMotor) ? ">> " : "   ";
                telemetry.addData(prefix + MOTOR_NAMES[i], "%.2f", drivetrain.getMotorScale(i));
            }
            telemetry.addLine("");
            telemetry.addData("Selected", MOTOR_NAMES[selectedMotor]);
            telemetry.addData("Scale range",
                    Constants.MOTOR_SCALE_MIN + " – " + Constants.MOTOR_SCALE_MAX
                    + "  (step " + Constants.MOTOR_SCALE_STEP + ")");
            telemetry.update();
        }
    }
}
