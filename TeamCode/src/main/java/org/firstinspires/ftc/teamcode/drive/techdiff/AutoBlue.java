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
    public static double R_ARM_STOW = 0.21;
    public static double R_ARM_GRAB = 0.7;
    public static double R_ARM_OVER = 0.58;
    public static double R_ARM_DROP = 0.35;

    public static double R_CLAW_STOW = 0.92;
    public static double R_CLAW_GRAB = 0.93;
    public static double R_CLAW_RELEASE = 0.15;

    public static double R_ROTATE_SIDE = 0.47;
    public static double R_ROTATE_BACK = 0;

    public static double FOUNDATION_GRAB = 0.75;
    public static double FOUNDATION_RELEASE = 0.5;

    ConstantInterpolator constInterp = new ConstantInterpolator(0);
    ConstantInterpolator constInterp180 = new ConstantInterpolator(Math.toRadians(180));
    LinearInterpolator linInterp = new LinearInterpolator(0,Math.toRadians(90));

    enum State {
        TO_FOUNDATION,
        TO_QUARRY,
        DEFAULT
    }

    // probably going to use an array for positions eventually
//    public static final double MIDDLE_STONE_X = -20;
//    public static final double SKYSTONE_OFFSET = -28;
    public static final double[] STONES_X = {-22, -29, -30, -38, -54, -58};
    // this offset is for intakes after the first one
    public static final double STONE_OFFSET = 0;

    private Servo rarm;
    private Servo rrotate;
    private Servo rclaw;
    private Servo larm;
    private Servo lclaw;
    private Servo lrotate;
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
        RsetRotate(R_ROTATE_SIDE);
        RsetArm(R_ARM_OVER);
        RsetClaw(R_CLAW_RELEASE);
        setFoundation(FOUNDATION_RELEASE);

        // First Pick-Up

        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(STONES_X[stonePos], 38))
                        .build()
                , State.DEFAULT);


        strafeAndGrabRight(drive, 4.5);

        drive.update();

        // Go to Foundation
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(10, 38))
                        .lineTo(new Vector2d(50, 40), linInterp)
                        .build()
                , State.TO_FOUNDATION);
        drive.update();

        //delay(2);

        // Move Foundation and deposit
        drive.followTrajectorySync(
                drive.trajectoryBuilderSlow()
                        .back(drive.getPoseEstimate().getY()-32)
                        .build()
        );
        setFoundation(FOUNDATION_GRAB);
        delay(0.7);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(24,55,Math.toRadians(180)))
                        .build()
        );
        setFoundation(FOUNDATION_RELEASE);
        deposit();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(7)
                        .back(20).build()
        );
        RsetArm(R_ARM_STOW);
        RsetClaw(R_CLAW_STOW);

        // Go back and Second Pick-Up
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(12, 36, Math.toRadians(180)), constInterp180)
                        .splineTo(new Pose2d(STONES_X[0],36,Math.toRadians(180)), constInterp180)
                        .build()
                , State.TO_QUARRY);
        strafeAndGrabLeft(drive, 5);
        drive.update();

        if (true) return;

        // Go back and Third Pick-Up
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(STONES_X[1],drive.getPoseEstimate().getY(),0), constInterp)
                        .build()
                , State.TO_QUARRY);
        drive.update();
//        strafeAndGrab(drive, 4);
        drive.update();

        // Go to foundation
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                .splineTo(new Pose2d(60, drive.getPoseEstimate().getY(), 0), constInterp)
                .build()
                , State.TO_FOUNDATION);
        drive.update();
//        deposit(drive, 5);

    }

    public void deposit() {
        RsetRotate(R_ROTATE_BACK);
        RsetArm(R_ARM_DROP);
        RsetClaw(R_CLAW_RELEASE);

    }

    public void strafeAndGrabRight(SampleMecanumDriveBase drive, double offset) {
        RsetArm(R_ARM_OVER);
        RsetClaw(R_CLAW_RELEASE);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(offset)
                        .build()
        );
        RsetArm(R_ARM_GRAB);
        delay(0.3);
        RsetClaw(R_CLAW_GRAB);
        delay(0.3);
        RsetArm(R_ARM_DROP);
//        delay(0.4);
//        RsetRotate(R_ROTATE_BACK);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(offset)
                        .build()
        );
    }

    public void strafeAndGrabLeft(SampleMecanumDriveBase drive, double offset) {
        RsetArm(R_ARM_OVER);
        RsetClaw(R_CLAW_RELEASE);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(offset)
                        .build()
        );
        RsetArm(R_ARM_GRAB);
        delay(0.3);
        RsetClaw(R_CLAW_GRAB);
        delay(0.3);
        RsetArm(R_ARM_DROP);
//        delay(0.4);
//        RsetRotate(R_ROTATE_BACK);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(offset)
                        .build()
        );
    }

    public void setFoundation(double pos) {
        foundation.setPosition(pos);
    }


    public void RsetArm(double p) {
        rarm.setPosition(p);
    }
    public void RsetClaw(double p) {
        rclaw.setPosition(p);
    }
    public void RsetRotate(double p) {
        rrotate.setPosition(p);
    }

    public void LsetArm(double p) {
        larm.setPosition(p);
    }
    public void LsetClaw(double p) {
        lclaw.setPosition(p);
    }
    public void LsetRotate(double p) {
        lrotate.setPosition(p);
    }

    public void delay(double time) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
            drive.update();
        }
    }

    // TODO: make state an argument, add a default case
    public void followTrajectoryArmSync(Trajectory t, State s) {
        drive.followTrajectory(t);
        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.update();
            switch(s) {
                case TO_FOUNDATION:
                    if(drive.getPoseEstimate().getX() > -40)
                        RsetRotate(R_ROTATE_BACK);
                    if(drive.getPoseEstimate().getX() > 20)
                        RsetArm(R_ARM_DROP);
                    break;
                case TO_QUARRY:
                    if(drive.getPoseEstimate().getX() > 15) {
                        RsetClaw(R_CLAW_STOW);
                        RsetArm(R_ARM_STOW);
                        RsetRotate(R_ROTATE_SIDE);
                    }
                    if(drive.getPoseEstimate().getX() < -12) {
                        RsetArm(R_ARM_OVER);
                        RsetClaw(R_CLAW_RELEASE);
                    }
                case DEFAULT:
                    break;
            }
        }
    }


}
