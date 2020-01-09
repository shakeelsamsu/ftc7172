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
public class AutoRed extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private static double R_ARM_STOW = 0.21;
    private static double R_ARM_GRAB = 0.7;
    private static double R_ARM_OVER = 0.58;
    private static double R_ARM_DROP = 0.35;

    private static double R_CLAW_STOW = 0.92;
    private static double R_CLAW_GRAB = 0.93;
    private static double R_CLAW_RELEASE = 0.15;

    private static double R_ROTATE_SIDE = 0.47;
    private static double R_ROTATE_DEPOSIT = 0.105;
    private static double R_ROTATE_BACK = 0;

    private static double FOUNDATION_GRAB = 0.75;
    private static double FOUNDATION_RELEASE = 0.5;

    //
    public static double L_ARM_STOW = 0.76;
    public static double L_ARM_GRAB = 0.3;
    public static double L_ARM_OVER = 0.38;
    public static double L_ARM_DROP = 0.62;

        // done
    public static double L_CLAW_STOW = 0;
    public static double L_CLAW_GRAB = 0;
    public static double L_CLAW_RELEASE = 0.65;

    public static double L_ROTATE_SIDE = 0.165;
    public static double L_ROTATE_DEPOSIT = 0.53;
    public static double L_ROTATE_BACK = 0.64;

    private static double ALLEY_Y = -39;

    ConstantInterpolator constInterp = new ConstantInterpolator(0);
    ConstantInterpolator constInterp180 = new ConstantInterpolator(Math.toRadians(-180));
    LinearInterpolator linInterp = new LinearInterpolator(0,Math.toRadians(-90));

    enum State {
        TO_FOUNDATION,
        TO_QUARRY,
        FINISH,
        DEFAULT
    }

    // probably going to use an array for positions eventually
//    public static final double MIDDLE_STONE_X = -20;
//    public static final double SKYSTONE_OFFSET = -28;
    public static final double[] STONES_X = {-29.5, -37.5, -42, -48, -54, -60};
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
        larm = hardwareMap.get(Servo.class, "larm");
        lrotate = hardwareMap.get(Servo.class, "lrotate");
        lclaw = hardwareMap.get(Servo.class, "lclaw");
        foundation = hardwareMap.get(Servo.class, "foundation");
        int stonePos = 5;

        while (!isStopRequested()) {
            if (gamepad1.x) {LsetArm(L_ARM_STOW);RsetArm(R_ARM_STOW);}
            else if (gamepad1.y) {LsetArm(L_ARM_GRAB);RsetArm(R_ARM_GRAB);}
            else if (gamepad1.a) {LsetArm(L_ARM_OVER);RsetArm(R_ARM_OVER);}
            else {LsetArm(L_ARM_DROP);RsetArm(R_ARM_DROP);}

            if (gamepad1.b) {LsetClaw(L_CLAW_GRAB);RsetClaw(R_CLAW_GRAB);}
            else if (gamepad1.dpad_up) {LsetClaw(L_CLAW_STOW);RsetClaw(R_CLAW_STOW);}
            else {LsetClaw(L_CLAW_RELEASE);RsetClaw(R_CLAW_RELEASE);}

            if (gamepad2.x) {LsetRotate(L_ROTATE_BACK);RsetRotate(R_ROTATE_BACK);}
            else if (gamepad2.y) {LsetRotate(L_ROTATE_SIDE);RsetRotate(R_ROTATE_SIDE);}
            else {LsetRotate(L_ROTATE_DEPOSIT);RsetRotate(R_ROTATE_DEPOSIT);}
        }

        waitForStart();

        drive.setPoseEstimate(new Pose2d(-36, -63, 0));
//        RsetRotate(R_ROTATE_SIDE);
//        RsetArm(R_ARM_OVER);
//        RsetClaw(R_CLAW_RELEASE);
        setFoundation(FOUNDATION_RELEASE);

        // First Pick-Up

        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(STONES_X[stonePos], -38))
                        .build()
                , State.DEFAULT);
        strafeAndGrabLeft(drive, 4.5);
        drive.update();

        // Go to Foundation
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(10, -38))
                        .lineTo(new Vector2d(50, -40), linInterp)
                        .build()
                , State.TO_FOUNDATION);
        drive.update();

        // Move Foundation and deposit
        drive.followTrajectorySync(
                drive.trajectoryBuilderSlow()
                        .back(Math.abs(drive.getPoseEstimate().getY()+30))
                        .build()
        );
        setFoundation(FOUNDATION_GRAB);
        delay(0.7);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(24,-55,Math.toRadians(-180)))
                        .build()
        );
        setFoundation(FOUNDATION_RELEASE);
        deposit();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(7)
                        .back(20).build()
        );
        RsetArm(R_ARM_STOW);
        RsetClaw(R_CLAW_STOW);
        // Go back and Second Pick-Up
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(12, ALLEY_Y, Math.toRadians(-180)), constInterp180)
                        .splineTo(new Pose2d(STONES_X[0],ALLEY_Y,Math.toRadians(-180)), constInterp180)
                        .build()
                , State.TO_QUARRY
        );
        delay(3);
        strafeAndGrabRight(drive, -drive.getPoseEstimate().getY()-33.5  );
        drive.update();

        //deposit second stone
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .reverse()
                .splineTo(new Pose2d(-10, ALLEY_Y, Math.toRadians(-180)))
                .splineTo(new Pose2d(49, ALLEY_Y, Math.toRadians(-180)))
                .build()
                , State.TO_FOUNDATION
        );
        deposit();
        delay(.25);

        // Go back and Third Pick-Up
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(12,ALLEY_Y,Math.toRadians(-180)))
                        .splineTo(new Pose2d(STONES_X[1],ALLEY_Y,Math.toRadians(-180)))
                        .build()
                , State.TO_QUARRY);
        drive.update();
        delay(3);
        strafeAndGrabRight(drive,-drive.getPoseEstimate().getY()-33.5);

        // Go to foundation 3
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(-10, ALLEY_Y, Math.toRadians(-180)))
                        .splineTo(new Pose2d(49, ALLEY_Y, Math.toRadians(-180)))
                        .build()
                , State.TO_FOUNDATION);
        drive.update();

        deposit();

        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                .splineTo(new Pose2d(4, ALLEY_Y, Math.toRadians(-180)))
                .build()
                , State.FINISH
        );
    }

    public void deposit() {
        RsetRotate(R_ROTATE_DEPOSIT);
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
        LsetArm(R_ARM_OVER);
        LsetClaw(R_CLAW_RELEASE);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(offset)
                        .build()
        );
        LsetArm(R_ARM_GRAB);
        delay(0.3);
        LsetClaw(R_CLAW_GRAB);
        delay(0.3);
        LsetArm(R_ARM_DROP);
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
                    if(drive.getPoseEstimate().getX() < 40)
                        RsetRotate(R_ROTATE_BACK);
                    if(drive.getPoseEstimate().getX() > -20)
                        RsetArm(R_ARM_DROP);
                    break;
                case TO_QUARRY:
                    if(drive.getPoseEstimate().getX() < -15) {
                        RsetClaw(R_CLAW_RELEASE);
                        RsetArm(R_ARM_OVER);
                        RsetRotate(R_ROTATE_SIDE);
                    }
                    break;
                case FINISH:
                    RsetArm(R_ARM_STOW);
                    delay(0.3);
                    RsetRotate(R_ROTATE_SIDE);
                    break;
                case DEFAULT:
                    break;
            }
        }
    }


}
