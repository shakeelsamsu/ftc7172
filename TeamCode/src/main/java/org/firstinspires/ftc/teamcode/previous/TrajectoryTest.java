package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
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
    public static Trajectories traj = new Trajectories();
    public static TrajectoryGenerator tg = TrajectoryGenerator.INSTANCE;
    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();

        drive.followTrajectorySync(traj.awayFromStones);

        LineSegment line = new LineSegment(new Vector2d(0,0), new Vector2d(0,3));
        LinearInterpolator interp = new LinearInterpolator(Math.toRadians(0),Math.toRadians(5));
        PathSegment segment = new PathSegment(line, interp);
        Path strafetest = new Path(segment);
        Trajectory traje = tg.generateTrajectory(strafetest, DriveConstants.BASE_CONSTRAINTS);

        drive.followTrajectorySync(traje);

    }
}