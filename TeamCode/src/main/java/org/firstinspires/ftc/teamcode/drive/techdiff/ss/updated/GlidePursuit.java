package org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.SSCurvePoint;
import org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.SSPoint;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.Localizer.world_heading;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.Localizer.world_x;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.Localizer.world_y;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.MathUtils.AngleWrap;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.MathUtils.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.MovementVars.movement_turn;

public class GlidePursuit {
//    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
//        double distToTarget = Math.hypot(x - world_x, y - world_y);
//
//        double absoluteAngleToTarget = Math.atan2(y - world_y, x - world_x);
//        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (world_heading - Math.toRadians(90)));
//
//        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distToTarget;
//        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distToTarget;
//
//        // Normalize the movement vectors
//        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
//        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
//
//        movementXPower *= movementSpeed;
//        movementYPower *= movementSpeed;
//
//        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
//        double movementHeadingPower = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
//
//        // Prevent heading overshoot when near target
//        if (distToTarget < 3) {
//            movementHeadingPower = 0;
//        }
//
//        // TODO: add pointAngle optimization and tinker with different smoothing effects near the end of a line
//
//        movementXPower *= Math.abs(relativeXToPoint) / 12;
//        movementYPower *= Math.abs(relativeYToPoint) / 12;
//
//        movementXPower = Range.clip(movementXPower, -movementSpeed, movementSpeed);
//        movementYPower = Range.clip(movementYPower, -movementSpeed, movementSpeed);
//
//        movement_x = movementXPower;
//        movement_y = movementYPower;
//        movement_turn = movementHeadingPower;
//
////        rmove(movementXPower, movementYPower, movementHeadingPower);
//
////        TelemetryPacket packet = new TelemetryPacket();
////
////        packet.put("world heading", world_heading);
////        packet.put("movement_turn", movement_turn);
////        packet.put("relative turn angle", relativeTurnAngle);
////        packet.put("relative turn angle / 30r", relativeTurnAngle / Math.toRadians(30));
////
////        dashboard.sendTelemetryPacket(packet);
//    }

    public static void goToPosition2(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distToTarget = Math.hypot(x - world_x, y - world_y);

        double absoluteAngleToTarget = Math.atan2(y - world_y, x - world_x);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (world_heading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distToTarget;

        // Normalize the movement vectors
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

//        movementXPower *= movementSpeed;
//        movementYPower *= movementSpeed;
        movementXPower *= Math.abs(relativeXToPoint) / 12;
        movementYPower *= Math.abs(relativeYToPoint) / 12;

        movement_x = Range.clip(movementXPower, -movementSpeed, movementSpeed);
        movement_y = Range.clip(movementYPower, -movementSpeed, movementSpeed);


        // Old Heading stuff
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
//        double movementHeadingPower = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

//         Prevent heading overshoot when near target
//        if (distToTarget < 3) {
//            movementHeadingPower = 0;
//        }

//        movement_x = movementXPower;
//        movement_y = movementYPower;

        // TODO: add pointAngle optimization and tinker with different smoothing effects near the end of a line
        // New heading stuff
        double actualRelativePointAngle = (preferredAngle - Math.toRadians(90));
        double angleToPointRaw = Math.atan2(y - world_y, x - world_x);
        double absolutePointAngle = angleToPointRaw + actualRelativePointAngle;

        double relativePointAngle = AngleWrap(absolutePointAngle - world_heading);

        double decelerationDistance = Math.toRadians(40);

        double movementHeadingPower = relativePointAngle*turnSpeed/decelerationDistance;

//        movementHeadingPower *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(30), 0, 1);
        movement_turn = Range.clip(movementHeadingPower, -turnSpeed,turnSpeed);

        if (distToTarget < 3) {
            movement_turn = 0;
        }

        movement_x *= Range.clip(Math.abs(relativeXToPoint/2.5),0,1);
        movement_y *= Range.clip(Math.abs(relativeYToPoint/2.5),0,1);
        movement_turn *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(2),0,1);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("world heading", Math.toDegrees(world_heading));
        packet.put("movement_turn", movement_turn);
        packet.put("relative point angle", Math.toDegrees(relativePointAngle));
        packet.put("absolute point angle", Math.toDegrees(absolutePointAngle));
//        packet.put("relative point angle / 2r", relativePointAngle / Math.toRadians(2));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
    }

    public static void displayTelemetry(Telemetry t) {
//        t.addData("world_x", world_x);
//        t.addData("world_y", world_y);
//        t.addData("world_heading", world_heading);
//        t.addData("currFollowIndex", closestToLine.index);
//        t.addData("currFollow x", closestToLine.x);
//        t.addData("currFollow y", closestToLine.y);
//        t.addData("follow x", followMe.x);
//        t.addData("follow y", followMe.y);
    }

    public static SSCurvePoint getFollowPointPath(ArrayList<SSCurvePoint> pathPoints, SSPoint robotLocation, double followRadius) {
//        followMe = new SSCurvePoint(pathPoints.get(currFollowIndex + 1));
//        followMe.setPoint(new SSPoint(closestToLine.x, closestToLine.y));

        // TODO: check this
        IndexedPoint closestToLine = findIndex(pathPoints, world_x, world_y);
        int currFollowIndex = closestToLine.index;

        SSCurvePoint followMe = new SSCurvePoint(pathPoints.get(currFollowIndex + 1));
        followMe.setPoint(new SSPoint(closestToLine.x, closestToLine.y));



        for (int i = 0; i < pathPoints.size() - 1; i++) {
            SSCurvePoint startLine = pathPoints.get(i);
            SSCurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<SSPoint> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = Integer.MAX_VALUE;
            for (SSPoint thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - world_y, thisIntersection.x - world_x);
                double deltaAngle = Math.abs(AngleWrap(angle - world_heading));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public static void followCurve(ArrayList<SSCurvePoint> allPoints, double followAngle) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        IndexedPoint closestToLine = findIndex(allPoints, world_x, world_y);
        int currFollowIndex = closestToLine.index + 1;

//        followMe = getFollowPointPath(allPoints, new SSPoint(world_x, world_y), allPoints.get(0).followDist);
        SSCurvePoint followMe = getFollowPointPath(allPoints, new SSPoint(world_x, world_y), allPoints.get(currFollowIndex).followDist);

        ArrayList<SSCurvePoint> pathExtended = (ArrayList<SSCurvePoint>) allPoints.clone();

        int numPoints = allPoints.size();
        pathExtended.set(pathExtended.size() - 1, extendLine(allPoints.get(numPoints - 1), allPoints.get(numPoints - 2), allPoints.get(numPoints - 1).pointLength));

        // Robot Position
        fieldOverlay.setFill("#3F51B5");
        fieldOverlay.fillRect(world_x, world_y, 7, 7);

        SSCurvePoint pointToMe = getFollowPointPath(pathExtended, new SSPoint(world_x, world_y), allPoints.get(currFollowIndex).followDist);

        SSCurvePoint lastPoint = allPoints.get(allPoints.size() - 1);
        double clippedDistance = Math.hypot(closestToLine.x - lastPoint.x, closestToLine.y - lastPoint.y);

        if(clippedDistance <= followMe.followDist || Math.hypot(world_x - lastPoint.x, world_y - lastPoint.y) < followMe.followDist + 15) {
            followMe.setPoint(lastPoint.toPoint());
        }

        for (int i = 0; i < allPoints.size(); i++) {
//            ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
//                new FloatPoint(allPoints.get(i + 1).x, allPoints.get(i + 1).y));

            // Path Lines
            fieldOverlay.setStroke("#FCDF03");
            if (i < allPoints.size() - 1)
                fieldOverlay.strokeLine(allPoints.get(i).x, allPoints.get(i).y, allPoints.get(i + 1).x, allPoints.get(i + 1).y);

            // Path Points
            fieldOverlay.setFill("#FF0000");
            fieldOverlay.fillCircle(allPoints.get(i).x, allPoints.get(i).y, 2);
        }

        // Following Point
//        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));
        fieldOverlay.setFill("#2CFC03");
        fieldOverlay.fillCircle(followMe.x, followMe.y, 1);

        // Pointing point
//        fieldOverlay.setFill("#03FC2C");
//        fieldOverlay.fillCircle(pointToMe.x, pointToMe.y, 1);

        double distToFinalEnd = Math.hypot(
                closestToLine.x-allPoints.get(allPoints.size()-1).x,
                closestToLine.y-allPoints.get(allPoints.size()-1).y);


        if(distToFinalEnd <= followMe.followDist + 4 ||
                Math.hypot(world_x-allPoints.get(allPoints.size()-1).x,
                        world_y-allPoints.get(allPoints.size()-1).y) < followMe.followDist + 4){

            followMe.setPoint(allPoints.get(allPoints.size()-1).toPoint());
        }

        // TODO: Figure out how to smooth out the followDist and stuff
        goToPosition2(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);


        // Testing
        double currFollowAngle = Math.atan2(pointToMe.y - world_y, pointToMe.x - world_x);
        currFollowAngle += AngleWrap(followAngle - Math.toRadians(90));

        double angleDelta = pointAngle(currFollowAngle, allPoints.get(currFollowIndex).turnSpeed, Math.toRadians(45));

        FtcDashboard dashboard = FtcDashboard.getInstance();

        packet.put("movement_x", movement_x);
        packet.put("movement_y", movement_y);
        packet.put("movement_turn", movement_turn);
        packet.put("world_x", world_x);
        packet.put("world_y", world_y);
        packet.put("world_turn", world_heading);

        dashboard.sendTelemetryPacket(packet);
    }

    // The relative point angle
    public static double pointAngle(double followAngle, double turnSpeed, double decelerationRadians) {
        double relativePointAngle = AngleWrap(followAngle - world_heading);

        double speed = (relativePointAngle/decelerationRadians)*turnSpeed;
        movement_turn = Range.clip(speed, -turnSpeed, turnSpeed);

        movement_turn *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(3), 0, 1);

        return AngleWrap(followAngle - world_heading);
    }

    public static IndexedPoint findIndex(ArrayList<SSCurvePoint> allPoints, double x, double y) {
        double closestDist = Integer.MAX_VALUE;

        int closestIndex = 0;

        SSPoint closestLine = new SSPoint();

        for (int i = 0; i < allPoints.size() - 1; i++) {
            SSCurvePoint pointA = allPoints.get(i);
            SSCurvePoint pointB = allPoints.get(i + 1);

            SSPoint currClosestToLine = findClosestPointOnLine(pointA.x, pointA.y, pointB.x, pointB.y, x, y);
            double distanceToClosest = Math.hypot(x - currClosestToLine.x, y - currClosestToLine.y);

            if (distanceToClosest < closestDist) {
                closestDist = distanceToClosest;
                closestIndex = i;
                closestLine = currClosestToLine;
            }
        }

        return new IndexedPoint(closestLine.x, closestLine.y, closestIndex);
    }

    public static SSPoint findClosestPointOnLine(double lineX1, double lineY1, double lineX2, double lineY2, double x, double y) {
        if (lineX1 == lineX2) {
            lineX1 = lineX2 + 0.01;
        }
        if (lineY1 == lineY2) {
            lineY1 = lineY2 + 0.01;
        }

        // Calculate the slope of the line
        double m1 = (lineY2 - lineY1) / (lineX2 - lineX1);
        // Calculate the slope perpendicular to this line
        double m2 = (lineX1 - lineX2) / (lineY2 - lineY1);

        double xClipedToLine = ((-m2 * x) + y + (m1 * lineX1) - lineY1) / (m1 - m2);
        double yClipedToLine = (m1 * (xClipedToLine - lineX1)) + lineY1;
        return new SSPoint(xClipedToLine, yClipedToLine);
    }

    public static SSCurvePoint extendLine(SSCurvePoint pointA, SSCurvePoint pointB, double additiveDistance) {

        double lineAngle = Math.atan2(pointB.y - pointA.y, pointB.x - pointA.x);

        double lineDistance = Math.hypot(pointB.y - pointA.y, pointB.x - pointA.x);

        double extendedLineLength = lineDistance + additiveDistance;

        SSCurvePoint extended = new SSCurvePoint(pointB);
        extended.x = Math.cos(lineAngle) * extendedLineLength + pointA.x;
        extended.y = Math.sin(lineAngle) * extendedLineLength + pointA.y;
        return extended;
    }

    static class IndexedPoint {
        double x;
        double y;
        int index;

        public IndexedPoint(double x, double y, int index) {
            this.x = x;
            this.y = y;
            this.index = index;
        }
    }
}
