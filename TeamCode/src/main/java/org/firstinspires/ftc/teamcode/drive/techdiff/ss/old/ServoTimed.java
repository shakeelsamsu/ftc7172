package org.firstinspires.ftc.teamcode.drive.techdiff.ss.old;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ServoTimed {
    Servo servo;
    ElapsedTime timer;
    public double stopTime = 0;
    public static double DURATION;
    public double min = 0;
    public double max = 1;

    public ServoTimed(Servo servo) {
        this.servo = servo;
        timer = new ElapsedTime();
    }

    public void setDuration(double duration) {
        DURATION = duration;
    }

    public double getDuration() {
        return DURATION;
    }

    public void setMin(double min) {
        this.min = min;
    }

    public void setMax(double max) {
        this.max = max;
    }

    public double getMin() {
        return min;
    }

    public double getMax() {
        return max;
    }

    public boolean isBusy() {
        return timer.seconds() < stopTime;
    }

    public void setPosition(double pos) {
        if(Math.abs(pos - servo.getPosition()) > 0.01)
            stopTime = timer.seconds() + DURATION;
        servo.setPosition(Range.clip(pos, min, max));
    }

    public double getPosition() {
        return servo.getPosition();
    }
}

