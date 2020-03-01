package org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated;

import android.os.SystemClock;

import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.SSMath.AngleWrap;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.Localizer.world_heading;

public class SpeedOmeter {
    private static long lastUpdateStartTime = 0;
    private static double currSpeedX = 0;
    private static double currSpeedY = 0;

    public static int timeBetweenUpdates = 25;
    public static double yDistTraveled = 0;
    public static double xDistTraveled = 0;

    public static double lastAngle = 0;

    public static double angularVelocity = 0;

    public static void update() {
        long currTime = SystemClock.uptimeMillis();

        if(currTime - lastUpdateStartTime > timeBetweenUpdates) {
            double elapsedTime = (double) (currTime - lastUpdateStartTime)/1000.00;
            double speedY = yDistTraveled / elapsedTime;
            double speedX = xDistTraveled / elapsedTime;

            if (speedY < 200 && speedX < 200) {
                currSpeedX = speedX;
                currSpeedY = speedY;
            }

            angularVelocity = AngleWrap(world_heading-lastAngle) / elapsedTime;
            lastAngle = world_heading;

            yDistTraveled = 0;
            xDistTraveled = 0;
            lastUpdateStartTime = currTime;
        }
    }

    public static double getSpeedY() {
        return currSpeedY;
    }
    public static double getSpeedX() {
        return currSpeedX;
    }
    public static double getDegPerSecond() {
        return Math.toDegrees(angularVelocity);
    }
    public static double getRadPerSecond() {
        return angularVelocity;
    }
    public static double ySlip1CMPS = 0.14;
    public static double xSlip1CMPS = 0.153;
    public static double turnSlip1RPS = 0.09;
    public static double currSlipDistanceY() {
        return SpeedOmeter.getSpeedY() * ySlip1CMPS;
    }
    public static double currSlipDistanceX() {
        return SpeedOmeter.getSpeedX() * xSlip1CMPS;
    }
    public static double currSlipAngle() {
        return getRadPerSecond() * turnSlip1RPS;
    }
}
