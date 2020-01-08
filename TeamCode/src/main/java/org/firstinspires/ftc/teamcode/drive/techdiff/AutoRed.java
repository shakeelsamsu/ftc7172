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
    public static double ARM_STOW = 0.21;
    public static double ARM_GRAB = 0.7;
    public static double ARM_OVER = 0.58;
    public static double ARM_DROP = 0.35;

    public static double CLAW_STOW = 0.92;
    public static double CLAW_GRAB = 0.93;
    public static double CLAW_RELEASE = 0.15;

    public static double ROTATE_SIDE = 0.47;
    public static double ROTATE_DEPOSIT = 0.105;
    public static double ROTATE_BACK = 0;

    public static double FOUNDATION_GRAB = 0.75;
    public static double FOUNDATION_RELEASE = 0.5;

    public static double ALLEY_Y = -39;

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
    public static final double[] STONES_X = {-29.5, -37.5, -42, -48, -54, -58};
    // this offset is for intakes after the first one
    public static final double STONE_OFFSET = 0;

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

        drive.setPoseEstimate(new Pose2d(-36, -63, 0));
//        setRotate(ROTATE_SIDE);
//        setArm(ARM_OVER);
//        setClaw(CLAW_RELEASE);
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
        setArm(ARM_STOW);
        setClaw(CLAW_STOW);
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
        setRotate(ROTATE_DEPOSIT);
        setArm(ARM_DROP);
        setClaw(CLAW_RELEASE);

    }

    public void strafeAndGrabRight(SampleMecanumDriveBase drive, double offset) {
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

    public void strafeAndGrabLeft(SampleMecanumDriveBase drive, double offset) {
        setArm(ARM_OVER);
        setClaw(CLAW_RELEASE);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(offset)
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
                        .strafeRight(offset)
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
                        setRotate(ROTATE_BACK);
                    if(drive.getPoseEstimate().getX() > -20)
                        setArm(ARM_DROP);
                    break;
                case TO_QUARRY:
                    if(drive.getPoseEstimate().getX() < -15) {
                        setClaw(CLAW_RELEASE);
                        setArm(ARM_OVER);
                        setRotate(ROTATE_SIDE);
                    }
                    break;
                case FINISH:
                    setArm(ARM_STOW);
                    delay(0.3);
                    setRotate(ROTATE_SIDE);
                    break;
                case DEFAULT:
                    break;
            }
        }
    }


}
