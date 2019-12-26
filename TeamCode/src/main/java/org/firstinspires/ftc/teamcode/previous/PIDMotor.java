package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@SuppressWarnings("unchecked")
public class PIDMotor {
    public final int IDLE = 0;
    public final int RUNNING_POWER = 1;
    public final int RUNNING_TO_TARGET = 2;
    public final int RUNNING_TO_LIMIT = 3;

    public DcMotorEx m0 = null;
    public DcMotorEx m1 = null;
    public DcMotor liftEnc = null;
    public AnalogInput zeroSwitch = null;
    
    public double power = 0;
    public double targetPower = 0;
    public int targetPosition;
    public int lastPosition;

    public double minPower = -0.4;
    public double maxPower = 0.4;
    
    public double kP;
    public double kI;
    public double kD;
    public double acc = 0;
    
    public int status = IDLE;
    
    public PIDMotor(DcMotorEx a, DcMotorEx b, DcMotor c, AnalogInput sw) {
        m0 = a;
        m1 = b;
        liftEnc = c;
        zeroSwitch = sw;
        setPIDCoefficients(0.020, 0, 0.004);
        setTargetPower(0);
    }
    
    public int getCurrentPosition() {
        return liftEnc.getCurrentPosition();
    }
    
    public double getPower() {
        return power;
    }
    
    public int getStatus() {
        return status;
    }
    
    public int getTargetPosition() {
        return targetPosition;
    }
    
    public double getTargetPower() {
        return targetPower;
    }
    
    public void resetPID() {
        lastPosition = getCurrentPosition();
        acc = 0;
    }
    
    public void setZeroPosition() {
        if (m0 != null) m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (m1 != null) m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (liftEnc != null) {
            liftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    
    public void setPowerLimits(double max) {
        minPower = -max;
        maxPower = max;
    }
    
    public void setPowerLimits(double min, double max) {
        minPower = min;
        maxPower = max;
    }
    
    public void setPIDCoefficients(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
    
    public void setTargetPosition(int target) {
        status = RUNNING_TO_TARGET;
        targetPower = 0;
        targetPosition = target;
        resetPID();
    }

    public void setTargetPower(double p) {
        targetPower = p;
        if (targetPower != 0) status = RUNNING_POWER;
    }

    public void setTargetLimit() { setTargetLimit(-999999); }

    public void setTargetLimit(int pos) {
        setTargetPosition(pos);
        status = RUNNING_TO_LIMIT;
    }

    public void update() {
        power = 0;
        if (zeroSwitch != null && zeroSwitch.getVoltage() < 1) {
            setZeroPosition();
            if (status == RUNNING_TO_LIMIT) status = IDLE;
        }
        switch (status) {
            case IDLE: power = 0; break;
            case RUNNING_POWER: power = targetPower; break;
            case RUNNING_TO_TARGET: power = calcPower(); break;
            case RUNNING_TO_LIMIT: power = calcPower(); break;
        }
        if (m0 != null) {
            m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m0.setPower(power);
        }
        if (m1 != null) {
            m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m1.setPower(power);
        }
    }
    
    public double calcPower() {
        int pos = getCurrentPosition();
        double error = targetPosition - pos;
        // error /= 22.76;
        acc = (acc + error) * 2.0 / 3.0;
        double p = kP * error;
        double i = kI * acc;
        double d = kD * (pos - lastPosition);
        lastPosition = pos;
        return Range.clip(p + i + d, minPower, maxPower);
    }

}
