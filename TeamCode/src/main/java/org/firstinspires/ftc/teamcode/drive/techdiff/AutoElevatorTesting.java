package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous
public class AutoElevatorTesting extends LinearOpMode {

    enum LiftState {
        HOME,
        DEPOSIT
    }
    LiftState ls = LiftState.HOME;

    DcMotor lift1;
    DcMotor lift2;
    ElapsedTime liftClock;
    static SampleMecanumDriveBase drive;

    public void runOpMode() {
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        liftClock = new ElapsedTime();

        waitForStart();
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        liftClock.reset();
        followTrajectoryLiftSync(drive.trajectoryBuilder().back(30).build());

    }

    public void followTrajectoryLiftSync(Trajectory t) {
        drive.followTrajectory(t);
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.update();
            liftUpdate();
            if(drive.getPoseEstimate().getX() < -15) {
                liftDeposit();
            }
        }
    }

    public void liftPower(double power) {
        lift1.setPower(power);
        lift2.setPower(power);
    }

    public void liftHome() {
        ls = LiftState.HOME;
        liftClock.reset();
    }

    public void liftDeposit() {
        ls = LiftState.DEPOSIT;
        liftClock.reset();
    }

    public void liftUpdate() {
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        switch(ls) {
            case HOME:
                if (liftClock.seconds() < 0.5 && lift1.getCurrentPosition() < -200)
                    liftPower(-0.5);
                else
                    liftPower(0);
                break;
            case DEPOSIT:
                if (liftClock.seconds() < 0.5 && lift1.getCurrentPosition() > -8000)
                    liftPower(0.5);
                else
                    liftPower(0);
                break;
        }
    }


}
