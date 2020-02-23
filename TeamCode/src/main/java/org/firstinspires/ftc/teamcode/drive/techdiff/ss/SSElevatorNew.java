package org.firstinspires.ftc.teamcode.drive.techdiff.ss;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SSElevatorNew {
    DcMotorEx lift1;
    DcMotorEx lift2;
    AnalogInput home;
    DcMotorEx enc;

    enum LiftState {
        HOME, DEPOSIT
    }

    LiftState ls;
    ElapsedTime liftClock;

    public SSElevatorNew(DcMotorEx lift1, DcMotorEx lift2, DcMotorEx enc, AnalogInput home) {
        this.lift1 = lift1;
        this.lift2 = lift2;
        this.home = home;
        this.enc = enc;
        liftClock = new ElapsedTime();
    }

    public void liftPower(double power) {
        lift1.setPower(power);
        lift2.setPower(power);
    }

    public void home() {
        ls = LiftState.HOME;
        liftClock.reset();
    }

    public void deposit() {
        ls = LiftState.DEPOSIT;
        liftClock.reset();
    }

    public void update() {
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        switch (ls) {
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
