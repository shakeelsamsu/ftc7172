package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.yaml.snakeyaml.scanner.Constant;


@Autonomous
@Config
public class TrajectoryTest extends LinearOpMode {
    public static double XD = 0;
    public static double YD = 20;
    public static TrajectoryGenerator tg = TrajectoryGenerator.INSTANCE;

    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();

//        Path strafetest = new PathBuilder(new Pose2d(0,0,0))
//                .lineTo(new Vector2d(0,20))
//                .build();
        LineSegment line = new LineSegment(new Vector2d(0,0), new Vector2d(0,20));
        ConstantInterpolator interp = new ConstantInterpolator(Math.toRadians(0));
        PathSegment segment = new PathSegment(line, interp);
        Path strafetest = new Path(segment);
        DriveConstraints constraints = new DriveConstraints(
                50.0, 45.0, 0.0,
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0
        );
        Trajectory traj = tg.generateTrajectory(strafetest, constraints);
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .strafeRight(20.0)
//                        .build()
//        );
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}