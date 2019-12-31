package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


@Autonomous
@Config
public class TrajectoryTest extends LinearOpMode {
    public static double XD = 0;
    public static double YD = 20;

    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();

        Path strafetest = new PathBuilder(new Pose2d(0,0,0))
                .lineTo(new Vector2d(0,20))
                .build();

        DriveConstraints constraints = new DriveConstraints(
                50.0, 45.0, 0.0,
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0
        );
        Trajectory traj = TrajectoryGenerator.generateTrajectory(strafetest, constraints);

        drive.followTrajectorySync(traj);
    }
}