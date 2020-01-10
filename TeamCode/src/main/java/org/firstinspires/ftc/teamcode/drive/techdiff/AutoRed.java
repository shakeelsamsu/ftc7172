package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous
public class AutoRed extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private static double R_ARM_STOW = 0.21;
    private static double R_ARM_GRAB = 0.66;
    private static double R_ARM_OVER = 0.58;
    private static double R_ARM_DROP = 0.35;

    public static double R_CLAW_STOW = 0.85;
    public static double R_CLAW_GRAB = 0.7;
    public static double R_CLAW_RELEASE = 0.17;

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
    public static double L_CLAW_STOW = 0.09;
    public static double L_CLAW_GRAB = 0.2;
    public static double L_CLAW_RELEASE = 0.75;

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
        TO_FINISH,
        DEFAULT
    }

    enum Claw {
        LEFT,
        RIGHT
    }
    // good: 0 1 5
    public static final double[] STONES_X = {-29.5, -37.5, -42, -48, -54, -60};
    public static final double[][] STONE_OPTIONS = {{5,0,1},{5,2,0},{4,1,0},{3,0,1}};

    private Servo rarm;
    private Servo rrotate;
    private Servo rclaw;
    private Servo larm;
    private Servo lclaw;
    private Servo lrotate;
    private Servo foundation;

    private DcMotor lift1, lift2;

    private SampleMecanumDriveBase drive;
    private State state;
    private Claw clawSide;
    private Claw CLAW_SIDE = clawSide.LEFT;
    private ElapsedTime liftClock = new ElapsedTime();

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
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int stonePos = 5;

        waitForStart();
        liftClock.reset();
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.setPoseEstimate(new Pose2d(-36, -63, 0));
        delay(1);
        RsetRotate(R_ROTATE_SIDE);
        RsetClaw(R_CLAW_STOW);
        RsetArm(R_ARM_STOW);
        LsetRotate(L_ROTATE_SIDE);
        LsetArm(L_ARM_OVER);
        LsetClaw(L_CLAW_RELEASE);
        setFoundation(FOUNDATION_RELEASE);

        // First Pick-Up
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(STONES_X[stonePos], -38))
                        .build()
                , State.DEFAULT
        );
        strafeAndGrab(drive, 4.5);
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
        followTrajectoryArmSync(
                drive.trajectoryBuilderSlow()
                        .back(Math.abs(drive.getPoseEstimate().getY()+30))
                        .build()
                , State.DEFAULT
        );
        setFoundation(FOUNDATION_GRAB);
        delay(0.7);
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(24,-55,Math.toRadians(-180)))
                        .build()
                , State.DEFAULT
        );
        setFoundation(FOUNDATION_RELEASE);
        deposit();
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .strafeRight(7)
                        .back(20).build()
                , State.DEFAULT
        );

        // Switch claw/arm sides
        CLAW_SIDE = clawSide.RIGHT;
        LsetRotate(L_ROTATE_SIDE);
        LsetArm(L_ARM_STOW);
        LsetClaw(L_CLAW_STOW);
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
        strafeAndGrab(drive, -drive.getPoseEstimate().getY()-33.5  );
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
        strafeAndGrab(drive,-drive.getPoseEstimate().getY()-33.5);

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
                , State.TO_FINISH
        );
    }

    public void deposit() {
        if (CLAW_SIDE == clawSide.RIGHT) {
            RsetRotate(R_ROTATE_DEPOSIT);
            RsetArm(R_ARM_DROP);
            RsetClaw(R_CLAW_RELEASE);
        }
        else {
            LsetRotate(L_ROTATE_DEPOSIT);
            LsetArm(L_ARM_DROP);
            LsetClaw(L_CLAW_RELEASE);
        }
    }
    public void strafeAndGrab(SampleMecanumDriveBase drive, double offset) {
        if (CLAW_SIDE == clawSide.LEFT) {
            LsetArm(L_ARM_OVER);
            LsetClaw(L_CLAW_RELEASE);
            followTrajectoryArmSync(
                    drive.trajectoryBuilder()
                            .strafeLeft(offset)
                            .build()
                    , State.DEFAULT
            );
            LsetArm(L_ARM_GRAB);
            delay(0.3);
            LsetClaw(L_CLAW_GRAB);
            delay(0.3);
            LsetArm(L_ARM_DROP);
            followTrajectoryArmSync(
                    drive.trajectoryBuilder()
                            .strafeRight(offset)
                            .build()
                    , State.DEFAULT
            );
        } else {
            RsetArm(R_ARM_OVER);
            RsetClaw(R_CLAW_RELEASE);
            followTrajectoryArmSync(
                    drive.trajectoryBuilder()
                            .strafeRight(offset)
                            .build()
                    , State.DEFAULT
            );
            RsetArm(R_ARM_GRAB);
            delay(0.3);
            RsetClaw(R_CLAW_GRAB);
            delay(0.3);
            RsetArm(R_ARM_DROP);
            followTrajectoryArmSync(
                    drive.trajectoryBuilder()
                            .strafeLeft(offset)
                            .build()
                    , State.DEFAULT
            );
        }
    }

    public void setFoundation(double pos) { foundation.setPosition(pos); }

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
            flipIntakeUpdate();
        }
    }

    public void liftPower(double pow) {
        lift1.setPower(pow);
        lift2.setPower(pow);
    }

    public void flipIntakeUpdate() {
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (liftClock.seconds() < 0.5 && lift1.getCurrentPosition() > -8000)  {
            liftPower(0.5);
        } else if (liftClock.seconds() <1.0) {
            liftPower(0);
        } else if (liftClock.seconds() < 5.0 && lift1.getCurrentPosition() < -200) {
            liftPower(-0.5);
        } else {
            liftPower(0);
        }
    }

    // TODO: make state an argument, add a default case
    public void followTrajectoryArmSync(Trajectory t, State s) {
        drive.followTrajectory(t);
        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.update();
            flipIntakeUpdate();
            switch(s) {
                case TO_FOUNDATION:
                    if(drive.getPoseEstimate().getX() > -40) {
                        if (CLAW_SIDE == clawSide.RIGHT) RsetRotate(R_ROTATE_BACK);
                        else LsetRotate(L_ROTATE_BACK);
                    }
                    break;
                case TO_QUARRY:
                    if(drive.getPoseEstimate().getX() < -15) {
                        RsetClaw(R_CLAW_RELEASE);
                        RsetArm(R_ARM_OVER);
                        RsetRotate(R_ROTATE_SIDE);
                    }
                    break;
                case TO_FINISH:
                    RsetRotate(R_ROTATE_SIDE);
                    delay(0.3);
                    RsetClaw(R_CLAW_STOW);
                    RsetArm(R_ARM_STOW);
                    break;
                case DEFAULT:
                    break;
            }
        }
    }


}
