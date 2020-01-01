package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class Trajectories {
    public static TrajectoryGenerator tg = TrajectoryGenerator.INSTANCE;

    LineSegment line = new LineSegment(new Vector2d(0,0), new Vector2d(0,-31));
    LinearInterpolator interp = new LinearInterpolator(Math.toRadians(0),Math.toRadians(5));
    PathSegment segment = new PathSegment(line, interp);
    Path strafetest = new Path(segment);
    public Trajectory middleStone = tg.generateTrajectory(strafetest, DriveConstants.BASE_CONSTRAINTS);

    LineSegment line2 = new LineSegment(new Vector2d(0,0), new Vector2d(-8,-31));
    ConstantInterpolator interp2 = new ConstantInterpolator(Math.toRadians(0));
    PathSegment segment2 = new PathSegment(line2, interp2);
    Path far = new Path(segment);
    public Trajectory farStone = tg.generateTrajectory(far, DriveConstants.BASE_CONSTRAINTS);

    LineSegment line3 = new LineSegment(new Vector2d(0,0), new Vector2d(8,-31));
    LinearInterpolator interp3 = new LinearInterpolator(Math.toRadians(0),Math.toRadians(5));
    PathSegment segment3 = new PathSegment(line3, interp3);
    Path close = new Path(segment3);
    public Trajectory closeStone = tg.generateTrajectory(strafetest, DriveConstants.BASE_CONSTRAINTS);

    LineSegment lin = new LineSegment(new Vector2d(0,0), new Vector2d(0,3));
    ConstantInterpolator inter = new ConstantInterpolator(Math.toRadians(0));
    PathSegment segmen = new PathSegment(lin, inter);
    Path away = new Path(segmen);
    public Trajectory awayFromStones = tg.generateTrajectory(strafetest, DriveConstants.BASE_CONSTRAINTS);
}
