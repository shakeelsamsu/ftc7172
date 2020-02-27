package org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit;

import android.graphics.Point;

public class SSCurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDist;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;

    public SSCurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDist, double pointLength, double slowDownTurnRadians, double slowDownTurnAmount) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDist = followDist;
        this.pointLength = pointLength;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public SSCurvePoint(SSCurvePoint p) {
        x = p.x;
        y = p.y;
        moveSpeed = p.moveSpeed;
        turnSpeed = p.turnSpeed;
        followDist = p.followDist;
        pointLength = p.pointLength;
        slowDownTurnRadians = p.slowDownTurnRadians;
        slowDownTurnAmount = p.slowDownTurnAmount;
    }

    public SSPoint toPoint() {
        return new SSPoint(x, y);
    }

    public void setPoint(SSPoint point) {
        x = point.x;
        y = point.y;
    }
}
