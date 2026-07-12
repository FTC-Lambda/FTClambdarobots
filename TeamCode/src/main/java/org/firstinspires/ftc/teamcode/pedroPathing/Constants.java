package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .forwardZeroPowerAcceleration(-34.6)   // Tuned: Forward Zero Power Acceleration
        .lateralZeroPowerAcceleration(-48.7);  // Tuned: Lateral Zero Power Acceleration

    public static MecanumConstants driveConstants = new MecanumConstants()
        .leftFrontMotorName("top_left")
        .leftRearMotorName("back_left")
        .rightFrontMotorName("top_right")
        .rightRearMotorName("back_right")
        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xVelocity(65.48)                      // Tuned: Forward Velocity (in/s)
        .yVelocity(-56.8);                     // Âm: robot strafe về trái khi Y dương (in/s)

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(-3.55) // TODO: Adjust based on your robot's measurements
        .strafePodX(0.91) // TODO: Adjust based on your robot's measurements
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("pinpoint") // Replace with actual hardware map name if different
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED); // Robot strafe đi trái = encoder đếm dương

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
