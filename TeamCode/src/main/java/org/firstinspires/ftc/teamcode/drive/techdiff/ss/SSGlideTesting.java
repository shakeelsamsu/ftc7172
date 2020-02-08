package org.firstinspires.ftc.teamcode.drive.techdiff.ss;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.drive.techdiff.GlideConstants;

public class SSGlideTesting {
    DcMotor lift1;
    DcMotor lift2;
    SampleMecanumDriveBase drive;
    ElapsedTime liftClock;
    ElapsedTime clock;

    public double GANTRY_RETRACT = SSGlideConstants.GANTRY_RETRACT;
    public double GANTRY_CAP = SSGlideConstants.GANTRY_CAP;
    public double GANTRY_EXTEND = SSGlideConstants.GANTRY_EXTEND;
    public static double GRAB_POS = SSGlideConstants.GRAB_POS;
    public static double LETGO_POS = SSGlideConstants.LETGO_POS;
    public static double STONE_DIST_BAR = SSGlideConstants.STONE_DIST_BAR;

    double gantryLock = 0;

    DistanceSensor stonedist = null;
    ServoTimed bar = null;
    Servo grab = null;
    Servo gantry = null;

    public final double BAR_FORWARD = 0.23;    // when bar is holding a stone
    public final double BAR_BACK = 0.75; // when bar points in robot's forward direction, when it is idle
    double barPos = BAR_FORWARD;

    DcMotor rin;
    DcMotor lin;
    double inPower = 0;

    enum LiftState {
        HOME,
        DEPOSIT
    }
    LiftState ls = LiftState.HOME;

    public void init(HardwareMap hardwareMap) {
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lin = hardwareMap.get(DcMotorEx.class, "lin");
        rin = hardwareMap.get(DcMotorEx.class, "rin");
        rin.setDirection(DcMotorSimple.Direction.FORWARD);
        lin.setDirection(DcMotorSimple.Direction.FORWARD);

        grab = hardwareMap.get(Servo.class, "grabber");
        gantry = hardwareMap.get(Servo.class, "gantry");

        bar = new ServoTimed(hardwareMap.get(Servo.class, "bar"));
        bar.setDuration(0.5);

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        liftClock = new ElapsedTime();
        clock = new ElapsedTime();
        clock.reset();
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

    public void inPower(double power) {
        rin.setPower(inPower = power);
        lin.setPower(-power);
    }

    public double stoneDist() {
        return stonedist.getDistance(DistanceUnit.INCH);
    }

    public void setBar(double pos) {
        bar.setPosition(pos);
    }

    public void setGantry(double pos) {
        if(Math.abs(gantry.getPosition() - pos) > 0.01) {
            gantryLock = clock.seconds() + GlideConstants.GANTRY_EXTEND_DELAY;
        }
        pos = Range.clip(pos,GANTRY_EXTEND,GANTRY_RETRACT);
        gantry.setPosition(pos);
    }

    public void setGrab(double pos) {
        pos = Range.clip(pos,GRAB_POS,LETGO_POS);
        grab.setPosition(pos);
    }

    public void grabBlock() {
        setGrab(GRAB_POS);
    }

    public void letGo() {
        setGrab(LETGO_POS);
    }

    public void stoneUpdate() {
        if(inPower < 0) {
            setBar(BAR_FORWARD);
            letGo();
            return;
        }
        if(ls == LiftState.HOME && stoneDist() < STONE_DIST_BAR) {
            setBar(BAR_BACK);
            setGantry(GANTRY_RETRACT);
            liftPower(-0.3);
            if(!bar.isBusy()) {
                grabBlock();
                liftPower(0);
            }
            else {
                letGo();
            }
        }
        else {
            setBar(BAR_FORWARD);
        }
    }

    public boolean grabbed() {
        return stoneDist() <= STONE_DIST_BAR && Math.abs(bar.getPosition() - BAR_BACK) <= 0.01;
    }
}
