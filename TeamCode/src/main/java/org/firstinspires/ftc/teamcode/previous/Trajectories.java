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

    // these stone names will probably change
    enum State {
        MIDDLE_STONE,
        FAR_STONE,
        CLOSE_STONE,
        TO_FOUNDATION,
        INTO_STONE
    }

    private LineSegment line;
    private LinearInterpolator lInterpolator;
    private ConstantInterpolator cInterpolator;
    private PathSegment segment;
    private Path path;

    public Trajectory getTrajectory(State s) {
        switch(s) {
            case MIDDLE_STONE:
                line = new LineSegment(new Vector2d(0,0), new Vector2d(-2,-32));
                lInterpolator = new LinearInterpolator(Math.toRadians(0),Math.toRadians(7.7));
                segment = new PathSegment(line, lInterpolator);
                path = new Path(segment);
                return tg.generateTrajectory(path, DriveConstants.BASE_CONSTRAINTS);
            case FAR_STONE:
                line = new LineSegment(new Vector2d(0,0), new Vector2d(-8,-31));
                lInterpolator = new LinearInterpolator(Math.toRadians(0),Math.toRadians(7.7));
                segment = new PathSegment(line, lInterpolator);
                path = new Path(segment);
                return tg.generateTrajectory(path, DriveConstants.BASE_CONSTRAINTS);
            case CLOSE_STONE:
                line = new LineSegment(new Vector2d(0,0), new Vector2d(8,-31));
                lInterpolator = new LinearInterpolator(Math.toRadians(0),Math.toRadians(7.7));
                segment = new PathSegment(line, lInterpolator);
                path = new Path(segment);
                return tg.generateTrajectory(path, DriveConstants.BASE_CONSTRAINTS);
            case TO_FOUNDATION:
                line = new LineSegment(new Vector2d(0,0), new Vector2d(-20,0));
                cInterpolator = new ConstantInterpolator(Math.toRadians(0));
                segment = new PathSegment(line, cInterpolator);
                path = new Path(segment);
                return tg.generateTrajectory(path, DriveConstants.BASE_CONSTRAINTS);
            case INTO_STONE:
                line = new LineSegment(new Vector2d(0,0), new Vector2d(0,-2));
                lInterpolator = new LinearInterpolator(Math.toRadians(0),Math.toRadians(-2));
                segment = new PathSegment(line, lInterpolator);
                path = new Path(segment);
                return tg.generateTrajectory(path, DriveConstants.BASE_CONSTRAINTS);
        }
        return null;
    }

    public LineSegment getLine() {
        return line;
    }

    public LinearInterpolator getLInterpolator() {
        return lInterpolator;
    }

    public ConstantInterpolator getCInterpolator() {
        return cInterpolator;
    }

    public PathSegment getSegment() {
        return segment;
    }

    public Path getPath() {
        return path;
    }

}
