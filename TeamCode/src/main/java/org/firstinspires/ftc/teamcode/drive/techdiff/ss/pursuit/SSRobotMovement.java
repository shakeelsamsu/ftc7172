package org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit;


import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.SSMath.AngleWrap;

public class SSRobotMovement {
    ThreeTrackingWheelLocalizer localizer;

    public SSRobotMovement(ThreeTrackingWheelLocalizer localizer) {
        localizer = this.localizer;
    }

    public void goToPosition(double x, double y, double speed) {
        double worldX = localizer.getPoseEstimate().getX();
        double worldY = localizer.getPoseEstimate().getY();
        double worldHeading = localizer.getPoseEstimate().getHeading();

        double distToTarget = Math.hypot(x-worldX, y - worldY);

        double absoluteAngleToTarget = Math.atan2(y - worldY, x - worldX);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldHeading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

    }
}
