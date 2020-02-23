package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoPark extends LinearOpMode {
    DcMotor lift1;
    DcMotor lift2;
    ElapsedTime liftClock;
    ElapsedTime timer;

    public void runOpMode() {
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftClock = new ElapsedTime();
        timer = new ElapsedTime();
        waitForStart();
        liftClock.reset();
        timer.reset();
        delay(24);
        liftClock.reset();
        while(opModeIsActive()) {
            flipIntakeUpdate();
        }
    }

    public void flipIntakeUpdate() {
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (liftClock.seconds() < 0.5 && lift1.getCurrentPosition() > -8000) {
            liftPower(0.5);
        } else if (liftClock.seconds() < 1.0) {
            liftPower(0);
        } else if (liftClock.seconds() < 5.0 && lift1.getCurrentPosition() < 200) {
            liftPower(-0.5);
        } else {
            liftPower(0);
        }
    }

    public void liftPower(double power) {
        lift1.setPower(power);
        lift2.setPower(power);
    }

    public void delay(double time) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
        }
    }
}
