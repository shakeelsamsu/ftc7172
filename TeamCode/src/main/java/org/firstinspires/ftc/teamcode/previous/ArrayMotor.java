package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.previous.PIDMotor;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class ArrayMotor extends PIDMotor {
    double[] powers;
    
    public ArrayMotor(DcMotorEx a, DcMotorEx b, DcMotor c,AnalogInput sw) {
        super(a, b, c,sw);
        powers = new double[301];
        int i = 0;
        while(i < 20) 
            powers[i++] = 0;
        while(i < 50)
            powers[i++] = 0.2;// + (i - 20) * 0.2 / 30;
        while(i < 100)
            powers[i++] = 0.5;
        while(i < 200)
            powers[i++] = 0.7;
        while(i < 300)
            powers[i++] = 0.9;
        powers[300] = 1;
    }
    
    public double calcPower() {
        int pos = getCurrentPosition();
        int error = targetPosition - pos;
        error /= 22.76;
        error = Range.clip(error, -300, 300);
        double ret = error > 0 ? powers[error] : -powers[-error];
        if (ret == 0 && pos > 24000) {
            ret = Math.sqrt(pos - 24000)*0.0006;
            ret = Range.clip(ret,0,.25);
        }
        return ret;
    } 
}
