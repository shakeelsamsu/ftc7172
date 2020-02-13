package org.firstinspires.ftc.teamcode.drive.mecanum;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.FASTER_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.FINISH_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MED_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.SLOW_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.FAST_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.FOUNDATION_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

/*
 * Base class with shared functionality for sample mecanum drives. All hardware-specific details are
 * handled in subclasses.
 */
@Config
public abstract class SampleMecanumDriveBase extends MecanumDrive {
    // 7172
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(2.8, 0, 0.22);
    public static PIDCoefficients LATERAL_PID = new PIDCoefficients(2.9, 0, 0.1);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(3, 0, 0); // switched I to D

//    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(2.8, 0, 0.22);
//    public static PIDCoefficients LATERAL_PID = new PIDCoefficients(5, 0, 0.1);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(12, 0, 0); // switched I to D


//    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0, 0, 0);
//    public static PIDCoefficients LATERAL_PID = new PIDCoefficients(0, 0, 0);
//    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0); // switched I to D

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    public Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private DriveConstraints constraintsSlow;
    private DriveConstraints constraintsFast;
    private DriveConstraints constraintsFaster;
    private DriveConstraints constraintsFinish;
    public DriveConstraints constraintsMed;
    public DriveConstraints constraintsFoundation;
    private TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;

    public SampleMecanumDriveBase() {
        super(kV, kA, kStatic, TRACK_WIDTH);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraintsFinish = new MecanumConstraints(FINISH_CONSTRAINTS ,TRACK_WIDTH);
        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        constraintsSlow = new MecanumConstraints(SLOW_CONSTRAINTS, TRACK_WIDTH);
        constraintsFast = new MecanumConstraints(FAST_CONSTRAINTS, TRACK_WIDTH);
        constraintsFaster = new MecanumConstraints(FASTER_CONSTRAINTS, TRACK_WIDTH);
        constraintsFoundation = new MecanumConstraints(FOUNDATION_CONSTRAINTS, TRACK_WIDTH);
        constraintsMed = new MecanumConstraints(MED_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(AXIAL_PID, LATERAL_PID, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public TrajectoryBuilder trajectoryBuilderSlow() {
        return new TrajectoryBuilder(getPoseEstimate(), constraintsSlow);
    }

    public TrajectoryBuilder trajectoryBuilderFinish() {
        return new TrajectoryBuilder(getPoseEstimate(), constraintsFinish);
    }

    public TrajectoryBuilder trajectoryBuilderFast() {
        return new TrajectoryBuilder(getPoseEstimate(), constraintsFast);
    }

    public TrajectoryBuilder trajectoryBuilderFaster() {
        return new TrajectoryBuilder(getPoseEstimate(), constraintsFaster);
    }

    public TrajectoryBuilder trajectoryBuilderFoundation() {
        return new TrajectoryBuilder(getPoseEstimate(), constraintsFoundation);
    }

    public TrajectoryBuilder trajectoryBuilderMed() {
        return new TrajectoryBuilder(getPoseEstimate(), constraintsMed);
    }

    public TrajectoryBuilder trajectoryBuilderTest(Trajectory prev, double t) {
        return new TrajectoryBuilder(prev, t, constraints);
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnSync(double angle) {
        turn(angle);
        waitForIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        followTrajectory(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("odo heading", Math.toDegrees(currentPose.getHeading()));
//        packet.put("imu heading", Math.toDegrees());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", Math.toDegrees(lastError.getHeading()));

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(currentPose.getHeading(), targetOmega);

                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());

                fieldOverlay.setStroke("#F44336");
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public List<Double> getWheelVelocities() {
        List<Double> positions = getWheelPositions();
        double currentTimestamp = clock.seconds();

        List<Double> velocities = new ArrayList<>(positions.size());;
        if (lastWheelPositions != null) {
            double dt = currentTimestamp - lastTimestamp;
            for (int i = 0; i < positions.size(); i++) {
                velocities.add((positions.get(i) - lastWheelPositions.get(i)) / dt);
            }
        } else {
            for (int i = 0; i < positions.size(); i++) {
                velocities.add(0.0);
            }
        }

        lastTimestamp = currentTimestamp;
        lastWheelPositions = positions;

        return velocities;
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);
}
