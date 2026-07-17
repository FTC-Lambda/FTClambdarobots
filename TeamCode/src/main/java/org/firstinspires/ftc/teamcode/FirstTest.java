package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "first_test", group = "Autonomous")
@Configurable // Panels
public class FirstTest extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ElapsedTime pathTimer = new ElapsedTime(); // Timer for delays between paths

    @Override
    public void init() {
        try {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        } catch (Exception e) {
            // Fallback nếu Panels chưa được khởi tạo
        }

        follower = Constants.createFollower(hardwareMap);
        
        // [QUAN TRỌNG] Đã cân chỉnh: Starting Pose PHẢI TRÙNG VỚI điểm bắt đầu của Path1! 
        // Code gốc của bạn để (72, 8) sẽ khiến robot bị giật cục lúc bắt đầu chạy.
        follower.setStartingPose(new Pose(33.550, 8.441, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Status", "Initialized");
            panelsTelemetry.update(telemetry);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Bắt đầu chạy path đầu tiên
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        if (panelsTelemetry != null) {
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }
        
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // Hàm setPathState để chạy Path tiếp theo
    public void setPathState(int state) {
        pathState = state;
        switch (state) {
            case 0:
                follower.followPath(paths.Path1);
                break;
            case 1:
                follower.followPath(paths.Path2);
                break;
            case 2:
                follower.followPath(paths.Path3);
                break;
            case 3:
                follower.followPath(paths.Path4);
                break;
            case 4:
                follower.followPath(paths.Path5);
                break;
            case 5:
                follower.followPath(paths.Path6);
                break;
            case 6:
                follower.followPath(paths.Path7);
                break;
            case 7:
                follower.followPath(paths.Path8);
                break;
        }
    }

    // Hàm cập nhật trạng thái tự động
    public void autonomousPathUpdate() {
        // Nếu follower ĐANG BẬN chạy, reset timer liên tục
        if (follower.isBusy()) {
            pathTimer.reset();
        } 
        // Nếu đã dừng chạy VÀ đã chờ đủ 1.0 giây
        else if (pathTimer.seconds() > 1.0) {
            if (pathState >= 0 && pathState < 8) {
                setPathState(pathState + 1);
            }
        }
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(33.550, 8.441),
                            new Pose(33.776, 34.413)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(225))
             .build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(33.776, 34.413),
                            new Pose(18.935, 35.376),
                            new Pose(22.935, 66.444)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(270))
             .build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(22.935, 66.444),
                            new Pose(40.102, 40.816)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(225))
             .build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(40.102, 40.816),
                            new Pose(18.393, 67.758),
                            new Pose(23.112, 89.743)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(270))
             .build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(23.112, 89.743),
                            new Pose(49.663, 50.852)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(225))
             .build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(49.663, 50.852),
                            new Pose(37.907, 92.793),
                            new Pose(11.339, 79.710)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(20))
             .build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(11.339, 79.710),
                            new Pose(37.357, 71.737),
                            new Pose(49.605, 51.187)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(225))
             .build();

            Path8 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(49.605, 51.187),
                            new Pose(39.216, 81.702),
                            new Pose(20.250, 79.676)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
             .build();
        }
    }
}
