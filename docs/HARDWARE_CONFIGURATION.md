# Hardware configuration

This is the source-of-truth checklist for the **names** expected in the Control Hub
or Robot Controller hardware map. The device configuration itself is stored on the
robot controller, not in this repository. The names are declared where they are used:
`hardware/RobotHardware.java` for the regular drivetrain and
`pedroPathing/Constants.java` for Pedro. Change both locations and the controller
configuration together when hardware is renamed.

## Required devices

| Hardware-map name | FTC device type | Used by | Notes |
| --- | --- | --- | --- |
| `top_left` | DC motor | standard drivetrain, Pedro | Physical front-left drive motor. |
| `back_left` | DC motor | standard drivetrain, Pedro | Physical rear-left drive motor. |
| `top_right` | DC motor | standard drivetrain, Pedro | Physical front-right drive motor. |
| `back_right` | DC motor | standard drivetrain, Pedro | Physical rear-right drive motor. |
| `limelight` | Limelight 3A | vision TeleOps and `AprilTag Seek` | USB-connected Limelight 3A. |
| `pinpoint` | goBILDA Pinpoint | Pedro tuning only | I2C device used by Pedro's Pinpoint localizer. |

The software sets all four Pedro drive directions to `FORWARD`. It does not set
motor direction, zero-power behavior, encoder mode, or port assignments in
`RobotHardware`; record and verify those choices in the controller configuration.

## Controller setup checklist

1. Configure the four drive motors and name them exactly as shown above.
2. Add the Limelight 3A as `limelight` and confirm its USB connection is healthy.
3. Add the goBILDA Pinpoint as `pinpoint` when using Pedro Pathing or its tuning
   OpMode. It is not needed by the ordinary or vision drivetrain OpModes.
4. Save the controller configuration, power-cycle the robot, then initialize
   `LambdaGoWild`. A missing name or wrong device type fails at initialization.
5. Run `Motor Calibration` after drive hardware changes. Its defaults are currently
   `top_left=1.00`, `back_left=1.40`, `top_right=1.00`, `back_right=1.40`.

## Limelight setup

Vision OpModes use pipeline **0**. Run `Limelight Pipeline Setup` once to make it an
FTC `aprilClassic36h11` fiducial pipeline, enable its 3D solve, and reset its crop to
the full sensor frame. The configured tag edge size is **165.1 mm**; measure the
printed black square and update both `LimelightPipelineSetup` and `AprilTagSeekAuto`
if that value differs. `AprilTag Seek` also attempts this repair during initialization.

## Current scope

No arm, intake, shooter, servo, or sensor is retrieved from the hardware map. Their
subsystem classes are placeholders, so do not add their controller names here until
code uses them.
