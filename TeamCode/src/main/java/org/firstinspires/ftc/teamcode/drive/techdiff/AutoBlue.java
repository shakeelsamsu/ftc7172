package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous
public class AutoBlue extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    public static double ARM_STOW = 0.21;
    public static double ARM_GRAB = 0.7;
    public static double ARM_OVER = 0.67;
    public static double ARM_DROP = 0.35;

    public static double CLAW_STOW = 0.92;
    public static double CLAW_GRAB = 0.93;
    public static double CLAW_RELEASE = 0.3;

    public static double ROTATE_SIDE = 0.47;
    public static double ROTATE_BACK = 0;

    enum State {
        TO_FOUNDATION,
        TO_QUARRY
    }

    // probably going to use an array for positions eventually
//    public static final double MIDDLE_STONE_X = -20;
//    public static final double SKYSTONE_OFFSET = -28;
    public static final double[] STONES_X = {-16, -22, -30, -38, -54, -62};

    private Servo rarm;
    private Servo rrotate;
    private Servo rclaw;

    private SampleMecanumDriveBase drive;
    private State state;

    public void runOpMode() {
        timer = new ElapsedTime();
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        rarm = hardwareMap.get(Servo.class, "rarm");
        rrotate = hardwareMap.get(Servo.class, "rrotate");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        int stonePos = 5;
        waitForStart();
        drive.setPoseEstimate(new Pose2d(-36, 63, 0));
        ConstantInterpolator interp = new ConstantInterpolator(0);
        setRotate(ROTATE_SIDE);
        setArm(ARM_OVER);
        setClaw(CLAW_RELEASE);

        // First Pick-Up
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        //.reverse()
                        .strafeTo(new Vector2d(STONES_X[stonePos], 36))
                        .build()//To(new Vector2d(36, -32)).build()
        );
        strafeAndGrab(drive, 4.5);
        drive.update();

        // Go to Foundation and Deposit 1
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .splineTo(new Pose2d(55, drive.getPoseEstimate().getY()),interp)
//                .build()
//        );
        state = State.TO_FOUNDATION;
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(55, drive.getPoseEstimate().getY()), interp)
                        .build()
        );
        deposit(drive,4.5);
        drive.update();

        // Go back and Second Pick-Up
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                    .reverse()
//                    //.addMarker(new Vector2d(drive.getPoseEstimate().getX(), 12), () -> {setRotate(ROTATE_BACK); setClaw(CLAW_RELEASE); setArm(ARM_OVER); return Unit.INSTANCE;})
//                    .splineTo(new Pose2d(STONES_X[stonePos + 3],drive.getPoseEstimate().getY()),interp)
//                .build()
//        );
        state = State.TO_QUARRY;
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(STONES_X[1],drive.getPoseEstimate().getY()),interp)
                        .build()
        );
        strafeAndGrab(drive, 5);
        drive.update();

        // Head to Foundation and Deposit 2
        state = State.TO_FOUNDATION;
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(60, drive.getPoseEstimate().getY(), 0))
                        .build()
        );
        deposit(drive,5);
        drive.update();

        delay(1000);

        // Go back and Third Pick-Up
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(STONES_X[0],drive.getPoseEstimate().getY()),interp)
                        .build()
        );
        drive.update();
    }
    public void deposit(SampleMecanumDriveBase drive, double offset) {
        setRotate(ROTATE_SIDE);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(offset)
                        .build()
        );
        setClaw(CLAW_RELEASE);
        delay(0.5);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(offset)
                        .build()
        );
        setArm(ARM_STOW);
        setClaw(CLAW_STOW);

    }
    public void strafeAndGrab(SampleMecanumDriveBase drive, double offset) {
        setArm(ARM_OVER);
        setClaw(CLAW_RELEASE);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(offset)
                        .build()
        );
        setArm(ARM_GRAB);
        delay(0.3);
        setClaw(CLAW_GRAB);
        delay(0.3);
        setArm(ARM_DROP);
//        delay(0.4);
//        setRotate(ROTATE_BACK);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(offset)
                        .build()
        );
    }
    public void setArm(double p) {
        rarm.setPosition(p);
    }
    public void setClaw(double p) {
        rclaw.setPosition(p);
    }
    public void setRotate(double p) {
        rrotate.setPosition(p);
    }
    public void delay(double time) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {}
    }

    // TODO: make state an argument, add a default case
    public void followTrajectoryArmSync(Trajectory t) {
        drive.followTrajectory(t);
        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.update();
            switch(state) {
                case TO_FOUNDATION:
                    if(drive.getPoseEstimate().getX() > -40)
                        setRotate(ROTATE_BACK);
                    break;
                case TO_QUARRY:
                    if(drive.getPoseEstimate().getX() > 15) {
                        setClaw(CLAW_STOW);
                        setArm(ARM_STOW);
                        setRotate(ROTATE_SIDE);
                    }
                    if(drive.getPoseEstimate().getX() < -12) {
                        setArm(ARM_OVER);
                        setClaw(CLAW_RELEASE);
                    }
            }


        }
    }


}
