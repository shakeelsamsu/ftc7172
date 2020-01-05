package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

    public static double FOUNDATION_GRAB = 0.1;
    public static double FOUNDATION_RELEASE = 0.5;

    ConstantInterpolator constInterp = new ConstantInterpolator(0);
    LinearInterpolator linInterp = new LinearInterpolator(0,Math.toRadians(90));

    enum State {
        TO_FOUNDATION,
        TO_QUARRY
    }

    // probably going to use an array for positions eventually
//    public static final double MIDDLE_STONE_X = -20;
//    public static final double SKYSTONE_OFFSET = -28;
    public static final double[] STONES_X = {-22, -29, -30, -38, -54, -62};

    private Servo rarm;
    private Servo rrotate;
    private Servo rclaw;
    private Servo foundation;

    private SampleMecanumDriveBase drive;
    private State state;

    public void runOpMode() {
        timer = new ElapsedTime();
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        rarm = hardwareMap.get(Servo.class, "rarm");
        rrotate = hardwareMap.get(Servo.class, "rrotate");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        foundation = hardwareMap.get(Servo.class, "foundation");
        int stonePos = 5;
        waitForStart();
        drive.setPoseEstimate(new Pose2d(-36, 63, 0));
        setRotate(ROTATE_SIDE);
        setArm(ARM_OVER);
        setClaw(CLAW_RELEASE);
        setFoundation(FOUNDATION_RELEASE);

        // First Pick-Up
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(STONES_X[stonePos], 36))
                        .build()
        );
        strafeAndGrab(drive, 4.5);
        drive.update();

                                                                                                        //go to foundation test
                                                                                                        drive.followTrajectorySync(
                                                                                                                drive.trajectoryBuilder()
                                                                                                                .splineTo(new Pose2d(12, drive.getPoseEstimate().getY(), 0), constInterp)
                                                                                                                .splineTo(new Pose2d(55, 42 , Math.toRadians(90)), linInterp)
                                                                                                                .build()
                                                                                                        );

        delay(100);

        // Go to Foundation and Deposit 1
        state = State.TO_FOUNDATION;
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(55, drive.getPoseEstimate().getY(), 0), constInterp)
                        .build()
        );
        deposit(drive,4.5);
        drive.update();

        // Go back and Second Pick-Up
        state = State.TO_QUARRY;
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(STONES_X[0],drive.getPoseEstimate().getY(),0), constInterp)
                        .build()
        );
        strafeAndGrab(drive, 5);
        drive.update();

        // Head to Foundation and Deposit 2
        state = State.TO_FOUNDATION;
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(60, drive.getPoseEstimate().getY(), 0), constInterp)
                        .build()
        );
        deposit(drive,4);
        drive.update();

        //delay(1000);

        // Go back and Third Pick-Up
        state = State.TO_QUARRY;
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(STONES_X[1],drive.getPoseEstimate().getY(),0), constInterp)
                        .build()
        );
        drive.update();
        strafeAndGrab(drive, 4);
        drive.update();

        // Go to foundation
        state = State.TO_FOUNDATION;
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                .splineTo(new Pose2d(60, drive.getPoseEstimate().getY(), 0), constInterp)
                .build()
        );
        drive.update();
        deposit(drive, 5);

        turnFoundation(drive);
    }
    public void turnFoundation(SampleMecanumDriveBase drive) {
        drive.turnSync(Math.toRadians(90));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(6)
                        .build()
        );
        setFoundation(FOUNDATION_GRAB);
        delay(0.7);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30,60,Math.toRadians(180)))
                        .build()
        );
        setFoundation(FOUNDATION_RELEASE);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(20).build()
        );

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
    public void setFoundation(double pos) {
        foundation.setPosition(pos);
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
