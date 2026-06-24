# Lambda Robotics Team

## Welcome to Lambda! 🚀

**Lambda** is a team of passionate Vietnamese engineers based in **Ho Chi Minh City (HCM)**, competing in the **FTC 2026-2027 season**.

This repository contains the robot control software for our competition robot. We are committed to innovation, teamwork, and pushing the boundaries of what's possible in robotics.

## Team Information

- **Team Name:** Lambda
- **Location:** Ho Chi Minh City, Vietnam
- **Season:** FIRST Tech Challenge 2026-2027
- **Focus:** Autonomous systems, hardware optimization, and competitive programming

## Getting Started

### Development Requirements

To work with this Android Studio project, you will need:

* **Android Studio Ladybug (2024.2)** or later
* **Java Development Kit (JDK)** compatible with your Android Studio version
* A basic understanding of Android development and FTC programming

### Project Structure

Our code is organized in the [/TeamCode](TeamCode) folder, which contains:

* **Auto** - Autonomous OpModes for match start
* **TeleOp** - Driver-controlled OpModes
* **Hardware** - Robot hardware abstraction and configuration
* **Subsystems** - Modular robot subsystems (Drivetrain, Arm, Intake, Shooter, etc.)
* **Util** - Utilities and constants

### Building and Running

1. Clone or download this repository
2. Open the project in Android Studio
3. Build the project using Gradle
4. Deploy to your Control Hub or Robot Controller device

## FTC Resources

For additional help and resources:

* [FIRST Tech Challenge Documentation](https://ftc-docs.firstinspires.org/index.html)
* [FTC Community Forum](https://ftc-community.firstinspires.org/)
* [FTC SDK Javadoc](https://javadoc.io/doc/org.firstinspires.ftc)

### Sample Code

The project includes FTC sample OpModes in [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples) that can be used as references for development.

## Team Development Guidelines

### Code Standards

* Follow Java naming conventions and best practices
* Use meaningful variable and method names
* Document complex algorithms and hardware interactions
* Keep methods focused and modular

### Subsystem Architecture

Our robot is organized into modular subsystems defined in [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems):

* **Drivetrain** - Movement and odometry
* **Arm** - Arm mechanism control
* **Intake** - Game piece intake system
* **Shooter** - Shooting mechanism

### Hardware Configuration

Robot hardware is configured in [RobotHardware.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/RobotHardware.java). Update this file when adding new components or changing device configurations.

### Constants

Team-specific constants such as motor speeds, servo positions, and timing values are defined in [Constants.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/Constants.java).

## Motor Calibration

Individual motors can drift over time due to wear, load differences, or manufacturing variance. The **Motor Calibration** OpMode lets you measure and correct for these differences without touching any code.

### How it works

Each motor has a scale factor (default `1.0`). The scale is applied **after** the standard drive normalization, so increasing a motor's scale gives it genuinely more power relative to the others. The valid range is `0.0` – `2.0`.

### Running the calibration

1. On the Driver Station, select **Motor Calibration** from the **Calibration** group.
2. Press **START**.

### Controls

| Input | Action |
|---|---|
| Left stick | Drive (forward / strafe) |
| Right stick | Rotate |
| D-pad UP | Select **Top-Left** motor |
| D-pad LEFT | Select **Back-Left** motor |
| D-pad RIGHT | Select **Top-Right** motor |
| D-pad DOWN | Select **Back-Right** motor |
| **RB** (right bumper) | Increase selected motor scale by 0.05 |
| **LB** (left bumper) | Decrease selected motor scale by 0.05 |
| **Y** | Reset all scales to 1.0 |

The telemetry screen shows all four scales live, with `>>` indicating the currently selected motor.

### Step-by-step procedure

1. Place the robot on a flat surface with plenty of room.
2. Start the OpMode and drive **straight forward** with the left stick.
3. Watch which direction the robot drifts.
   - Drifts **left** → the left-side motors are slightly weaker. Select **Top-Left** and **Back-Left** one at a time and press **RB** to increase their scale by small steps (e.g. +0.05).
   - Drifts **right** → do the same for the right-side motors.
4. Re-test after each adjustment until the robot drives straight.
5. Note the final scale values from the telemetry display and record them permanently in `Constants.java` or `Drivetrain.java` as the new defaults so they survive OpMode restarts.

## Competitions

We compete in the **2026-2027 FTC Season** and are dedicated to:

* Developing efficient autonomous routines
* Precise TeleOp control systems
* Reliable hardware integration
* Strategic gameplay optimization

---

**Go Lambda! 💪**
