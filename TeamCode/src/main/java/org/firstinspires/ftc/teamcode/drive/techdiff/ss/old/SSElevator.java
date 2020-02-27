package org.firstinspires.ftc.teamcode.drive.techdiff.ss.old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;


public class SSElevator {

    DcMotorEx lift1;
    DcMotorEx lift2;
    DcMotorEx enc;
    AnalogInput home;

    double targetPower = 0;
    int targetPosition = 0;
    double power = 0;
    double pidPower = 0;
    ElapsedTime pidClock = null;

    public double iTerm = 0;
    int lastPos = 0;

    enum State {
        IDLE,
        RUNNING_POWER,
        RUNNING_TO_HOME,
        RUNNING_TO_TARGET
    }

    State state = State.IDLE;

    public SSElevator(DcMotorEx lift1, DcMotorEx lift2, DcMotorEx enc, AnalogInput home) {
        this.lift1 = lift1;
        this.lift2 = lift2;
        this.enc = enc;
        this.home = home;
        this.pidClock = new ElapsedTime();
    }

    public int getCurrentPosition() {
        return -enc.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public double getPower() {
        return power;
    }

    public double getTargetPower() {
        return calcPower();
    }

    public String getState() {
        return state.toString();
    }

    public double holdPower() {
        return getCurrentPosition() > 30000 ? 0.1 : 0;
    }

    public double downPower() {
        int pos = getCurrentPosition();
        return Range.clip((pos - 30000) / 100000d, -0.2, 0);
    }

    public void setTargetPosition(int target) {
        targetPosition = target;
        targetPower = 0;
        state = State.RUNNING_TO_TARGET;
    }

    public void setTargetHome() {
        setTargetPosition(-99999);
        state = State.RUNNING_TO_HOME;
    }

    public void setZeroPosition() {
        if (lift1 != null) lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (lift2 != null) lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (enc != null) {
            enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            enc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setTargetPower(double power) {
        targetPower = power;
        if(targetPower != 0) state = State.RUNNING_POWER;
    }

    public void update() {
        power = 0;
        if (home != null && home.getVoltage() < 1) {
            setZeroPosition();
            if (state == State.RUNNING_TO_HOME) state = State.IDLE;
        }
        switch (state) {
            case IDLE: break;
            case RUNNING_POWER: power = targetPower; break;
            case RUNNING_TO_TARGET: power = calcPower(); break;
            case RUNNING_TO_HOME: power = calcPower(); break;
        }
        // if(power > -0.01 && power < 0.01) power = holdPower();
        if (lift1 != null) {
            lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift1.setPower(power);
        }
        if (lift2 != null) {
            lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift2.setPower(power);
        }
    }

    public double calcPower() {
        // limit sampling to 100 Hz
        if (pidClock.seconds() < 0.01) return pidPower;
        pidClock.reset();
        double kP = 0.0002;
        double kI = 0.000015;
        double kD = 0.00000;

        int pos = getCurrentPosition();
        double ePos = getTargetPosition() - pos;
        double dPos = lastPos - pos;
        double fTerm = 0; // ePos > 300 ? 0.5 : 0;
        double pTerm = kP * ePos;
        double dTerm = -kD * dPos;
        if (ePos < -2000 || ePos > 1500) iTerm = 0;
        else { iTerm = iTerm + kI * ePos; }
        iTerm = Range.clip(iTerm, -0.3, 0.5);

        lastPos = pos;
        pidPower = Range.clip(pTerm + iTerm + dTerm + fTerm, -1.0, 1.0);
        return pidPower;
    }
}
