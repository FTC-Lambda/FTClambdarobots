package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.PIDController;

/**
 * Diagnostic OpMode to run all drivetrain motors at a target velocity using PID.
 *
 * Controls:
 *   R1           = run motors
 *   D-pad Up    = previous parameter
 *   D-pad Down  = next parameter
 *   D-pad Left  = decrease parameter
 *   D-pad Right = increase parameter
 *   A            = decrease step size
 *   B            = increase step size
 *
 * NOTE:
 * PIDController uses:
 *      error = measurement - setpoint
 *
 * Therefore this OpMode passes:
 *      measurement = -velocity
 *      setpoint    = -target
 *
 * so that positive PID output corresponds to increasing motor power.
 */
@TeleOp(name = "Max RPM PID Tuner", group = "Test")
public class MaxRpmTest extends LinearOpMode {

    private enum Parameter {
        KP, KI, KD, TARGET
    }

    private Parameter selectedParam = Parameter.KP;

    // ---------------------------------------------------------
    // PID / TARGET SETTINGS
    // ---------------------------------------------------------

    private double kp = 0.001;
    private double ki = 0.0001;
    private double kd = 0.00005;

    // Encoder ticks per second
    private double targetTps = 2500.0;

    private double step = 0.0001;

    // ---------------------------------------------------------
    // BUTTON STATE
    // ---------------------------------------------------------

    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        // ---------------------------------------------------------
        // MOTOR REFERENCES
        // ---------------------------------------------------------

        DcMotorEx topLeft = (DcMotorEx) robot.topLeft;
        DcMotorEx topRight = (DcMotorEx) robot.topRight;
        DcMotorEx backLeft = (DcMotorEx) robot.backLeft;
        DcMotorEx backRight = (DcMotorEx) robot.backRight;

        DcMotorEx[] motors = {
                topLeft,
                topRight,
                backLeft,
                backRight
        };

        // ---------------------------------------------------------
        // MOTOR DIRECTIONS
        // ---------------------------------------------------------

        // Match drivetrain forward direction.
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // ---------------------------------------------------------
        // MOTOR CONFIGURATION
        // ---------------------------------------------------------

        for (DcMotorEx motor : motors) {

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.FLOAT
            );

            motor.setPower(0);
        }

        // ---------------------------------------------------------
        // PID CONTROLLERS
        // ---------------------------------------------------------

        PIDController pidTL =
                new PIDController(kp, ki, kd, 1.0, 100, 0.5);

        PIDController pidTR =
                new PIDController(kp, ki, kd, 1.0, 100, 0.5);

        PIDController pidBL =
                new PIDController(kp, ki, kd, 1.0, 100, 0.5);

        PIDController pidBR =
                new PIDController(kp, ki, kd, 1.0, 100, 0.5);

        // Keep track of which gains the PID objects currently contain.
        double activeKp = kp;
        double activeKi = ki;
        double activeKd = kd;

        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("=== MAX RPM PID TUNER ===");
        telemetry.addLine("Hold R1 to run");
        telemetry.addLine("Release R1 to tune");
        telemetry.update();

        waitForStart();

        timer.reset();

        while (opModeIsActive()) {

            // -----------------------------------------------------
            // DELTA TIME
            // -----------------------------------------------------

            double dt = timer.seconds();
            timer.reset();

            if (dt <= 0) {
                dt = 0.001;
            }

            // -----------------------------------------------------
            // INPUT
            // -----------------------------------------------------

            handleInput();

            boolean running = gamepad1.right_bumper;

            // -----------------------------------------------------
            // UPDATE PID COEFFICIENTS
            // -----------------------------------------------------
            //
            // PIDController's gains are final, so we need new
            // instances when the user changes them.
            //
            // We ONLY do this while stopped so that running PID
            // state is never destroyed.
            // -----------------------------------------------------

            if (!running &&
                    (kp != activeKp ||
                            ki != activeKi ||
                            kd != activeKd)) {

                pidTL = new PIDController(
                        kp, ki, kd, 1.0, 100, 0.5
                );

                pidTR = new PIDController(
                        kp, ki, kd, 1.0, 100, 0.5
                );

                pidBL = new PIDController(
                        kp, ki, kd, 1.0, 100, 0.5
                );

                pidBR = new PIDController(
                        kp, ki, kd, 1.0, 100, 0.5
                );

                activeKp = kp;
                activeKi = ki;
                activeKd = kd;
            }

            // -----------------------------------------------------
            // MOTOR VELOCITY
            // -----------------------------------------------------

            double velTL = topLeft.getVelocity();
            double velTR = topRight.getVelocity();
            double velBL = backLeft.getVelocity();
            double velBR = backRight.getVelocity();

            double pTL = 0;
            double pTR = 0;
            double pBL = 0;
            double pBR = 0;

            // -----------------------------------------------------
            // RUN / STOP
            // -----------------------------------------------------

            if (running) {

                /*
                 * Your PIDController uses:
                 *
                 *     error = measurement - setpoint
                 *
                 * For a velocity controller we want:
                 *
                 *     error = target - velocity
                 *
                 * Passing both values as negatives gives that result:
                 *
                 *     (-velocity) - (-target)
                 *     = target - velocity
                 */

                pTL = pidTL.calculate(
                        -velTL,
                        -targetTps,
                        dt
                );

                pTR = pidTR.calculate(
                        -velTR,
                        -targetTps,
                        dt
                );

                pBL = pidBL.calculate(
                        -velBL,
                        -targetTps,
                        dt
                );

                pBR = pidBR.calculate(
                        -velBR,
                        -targetTps,
                        dt
                );

                topLeft.setPower(clamp(pTL));
                topRight.setPower(clamp(pTR));
                backLeft.setPower(clamp(pBL));
                backRight.setPower(clamp(pBR));

            } else {

                topLeft.setPower(0);
                topRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                pidTL.reset();
                pidTR.reset();
                pidBL.reset();
                pidBR.reset();
            }

            // -----------------------------------------------------
            // TELEMETRY
            // -----------------------------------------------------

            telemetry.addLine("=== PID TUNER ===");

            telemetry.addData(
                    "R1",
                    gamepad1.right_bumper
                            ? "PRESSED"
                            : "NOT PRESSED"
            );

            telemetry.addData(
                    "Status",
                    running
                            ? "RUNNING"
                            : "STOPPED - HOLD R1"
            );

            telemetry.addData(
                    "Selected",
                    selectedParam
            );

            telemetry.addData(
                    "Step",
                    "%.6f",
                    step
            );

            telemetry.addData(
                    selectedParam == Parameter.KP
                            ? "> KP"
                            : "  KP",
                    "%.6f",
                    kp
            );

            telemetry.addData(
                    selectedParam == Parameter.KI
                            ? "> KI"
                            : "  KI",
                    "%.6f",
                    ki
            );

            telemetry.addData(
                    selectedParam == Parameter.KD
                            ? "> KD"
                            : "  KD",
                    "%.6f",
                    kd
            );

            telemetry.addData(
                    selectedParam == Parameter.TARGET
                            ? "> TARGET"
                            : "  TARGET",
                    "%.0f TPS",
                    targetTps
            );

            telemetry.addLine("=== MOTOR SPEED ===");

            telemetry.addData(
                    "Top Left",
                    "V: %.0f | P: %.3f",
                    velTL,
                    pTL
            );

            telemetry.addData(
                    "Top Right",
                    "V: %.0f | P: %.3f",
                    velTR,
                    pTR
            );

            telemetry.addData(
                    "Back Left",
                    "V: %.0f | P: %.3f",
                    velBL,
                    pBL
            );

            telemetry.addData(
                    "Back Right",
                    "V: %.0f | P: %.3f",
                    velBR,
                    pBR
            );

            telemetry.update();
        }

        // ---------------------------------------------------------
        // SAFETY SHUTDOWN
        // ---------------------------------------------------------

        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    // =============================================================
    // INPUT HANDLING
    // =============================================================

    private void handleInput() {

        // ---------------------------------------------------------
        // PARAMETER SELECTION
        // ---------------------------------------------------------

        if (gamepad1.dpad_up && !lastUp) {

            int next =
                    (selectedParam.ordinal()
                            - 1
                            + Parameter.values().length)
                            % Parameter.values().length;

            selectedParam = Parameter.values()[next];
        }

        if (gamepad1.dpad_down && !lastDown) {

            int next =
                    (selectedParam.ordinal() + 1)
                            % Parameter.values().length;

            selectedParam = Parameter.values()[next];
        }

        // ---------------------------------------------------------
        // PARAMETER ADJUSTMENT
        // ---------------------------------------------------------

        double direction = 0;

        if (gamepad1.dpad_left && !lastLeft) {
            direction = -1;
        }

        if (gamepad1.dpad_right && !lastRight) {
            direction = 1;
        }

        if (direction != 0) {

            switch (selectedParam) {

                case KP:
                    kp += direction * step;
                    break;

                case KI:
                    ki += direction * step;
                    break;

                case KD:
                    kd += direction * step;
                    break;

                case TARGET:
                    targetTps += direction * step * 1000;
                    break;
            }

            // Prevent negative gains/target
            kp = Math.max(0, kp);
            ki = Math.max(0, ki);
            kd = Math.max(0, kd);
            targetTps = Math.max(0, targetTps);
        }

        // ---------------------------------------------------------
        // STEP SIZE
        // ---------------------------------------------------------

        if (gamepad1.b && !lastB) {
            step *= 10;
        }

        if (gamepad1.a && !lastA) {
            step /= 10;
        }

        // ---------------------------------------------------------
        // SAVE BUTTON STATES
        // ---------------------------------------------------------

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
    }

    // =============================================================
    // CLAMP
    // =============================================================

    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}