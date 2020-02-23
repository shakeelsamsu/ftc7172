package org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.company.FloatPoint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.gfdebug.ComputerDebugging;

import java.util.ArrayList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.SSMath.AngleWrap;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.SSMath.lineCircleIntersection;

public class SSGlideP {
    ThreeTrackingWheelLocalizer localizer;
    DcMotor rf;
    DcMotor rb;
    DcMotor lf;
    DcMotor lb;

    int currFollowIndex = 0;
    IndexedPoint closestToLine;
    SSCurvePoint followMe;

    private FtcDashboard dashboard;
    private NanoClock clock;

    public SSGlideP() {
        movement_x = 0;
        movement_y = 0;
        movement_turn = 0;

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();
    }

    /**
     * Initializes Glide's hardware
     * @param hardwareMap the HardwareMap object
     */
    public void init(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
    }

    /**
     * Moves the robot
     * @param xPower the x-component of the power
     * @param yPower the y-component of the power
     * @param headingPower the heading-component of the power
     */
    public void rmove(double xPower, double yPower, double headingPower) {
        lf.setPower(yPower+xPower-headingPower);
        rf.setPower(yPower-xPower+headingPower);
        lb.setPower(yPower-xPower-headingPower);
        rb.setPower(yPower+xPower+headingPower);
    }


    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        localizer.update();

        double worldX = localizer.getPoseEstimate().getX();
        double worldY = localizer.getPoseEstimate().getY();
        double worldHeading = localizer.getPoseEstimate().getHeading();

        double distToTarget = Math.hypot(x - worldX, y - worldY);

        double absoluteAngleToTarget = Math.atan2(y - worldY, x - worldX);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldHeading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distToTarget;

        // Normalize the movement vectors
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movementXPower *= movementSpeed;
        movementYPower *= movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        double movementHeadingPower = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        // Prevent heading overshoot when near target
        if(distToTarget < 3) {
            movementHeadingPower = 0;
        }

        movement_x = movementXPower;
        movement_y = movementYPower;
        movement_turn = movementHeadingPower;
        rmove(movementXPower, movementYPower, movementHeadingPower);
    }

    public void displayTelemetry(Telemetry t) {
        t.addData("worldX", localizer.getPoseEstimate().getX());
        t.addData("worldY", localizer.getPoseEstimate().getY());
        t.addData("worldHeading", localizer.getPoseEstimate().getHeading());
        t.addData("currFollowIndex", closestToLine.index);
//        t.addData("currFollow x", closestToLine.x);
//        t.addData("currFollow y", closestToLine.y);
        t.addData("follow x", followMe.x);
        t.addData("follow y", followMe.y);
    }

    public SSCurvePoint getFollowPointPath(ArrayList<SSCurvePoint> pathPoints, SSPoint robotLocation, double followRadius) {
        double worldX = localizer.getPoseEstimate().getX();
        double worldY = localizer.getPoseEstimate().getY();
        double worldHeading = localizer.getPoseEstimate().getHeading();

        closestToLine = findIndex(pathPoints, worldX, worldY);
        currFollowIndex = closestToLine.index;


//        followMe = new SSCurvePoint(pathPoints.get(currFollowIndex + 1));
//        followMe.setPoint(new SSPoint(closestToLine.x, closestToLine.y));

        followMe = new SSCurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++) {
            SSCurvePoint startLine = pathPoints.get(i);
            SSCurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<SSPoint> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = Integer.MAX_VALUE;
            for(SSPoint thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - worldY, thisIntersection.x - worldX);
                double deltaAngle = Math.abs(AngleWrap(angle - worldHeading));

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public void followCurve(ArrayList<SSCurvePoint> allPoints, double followAngle) {
        update();
        double worldX = localizer.getPoseEstimate().getX();
        double worldY = localizer.getPoseEstimate().getY();
        double worldHeading = localizer.getPoseEstimate().getHeading();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setFill("#3F51B5");
        fieldOverlay.fillRect(worldX, worldY, 7, 7);

        for(int i = 0; i < allPoints.size() - 1; i++) {
//            ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
//                new FloatPoint(allPoints.get(i + 1).x, allPoints.get(i + 1).y));
            // Insert Debugging Code Here eventually
            fieldOverlay.setFill("#FF0000");
            fieldOverlay.fillCircle(allPoints.get(i).x, allPoints.get(i).y, 2);
        }

//        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

        SSCurvePoint followMe = getFollowPointPath(allPoints, new SSPoint(worldX, worldY), allPoints.get(0).followDist);

        // Only uses the first follow distance
        // TODO: Figure out how to smooth out the followDist and stuff
//        SSCurvePoint followMe = getFollowPointPath(allPoints, new SSPoint(worldX, worldY), allPoints.get(currFollowIndex + 1).followDist);
        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);

        dashboard.sendTelemetryPacket(packet);
    }

    public IndexedPoint findIndex(ArrayList<SSCurvePoint> allPoints, double x, double y) {
        double closestDist = Integer.MAX_VALUE;

        int closestIndex = 0;

        SSPoint closestLine = new SSPoint();

        for(int i = 0; i < allPoints.size() - 1; i++) {
            SSCurvePoint pointA = allPoints.get(i);
            SSCurvePoint pointB = allPoints.get(i + 1);

            SSPoint currClosestToLine = findClosestPointOnLine(pointA.x, pointA.y, pointB.x, pointB.y, x, y);
            double distanceToClosest = Math.hypot(x - currClosestToLine.x, y - currClosestToLine.y);

            if(distanceToClosest < closestDist) {
                closestDist = distanceToClosest;
                closestIndex = i;
                closestLine = currClosestToLine;
            }
        }

        return new IndexedPoint(closestLine.x, closestLine.y, closestIndex);
    }

    public SSPoint findClosestPointOnLine(double lineX1, double lineY1, double lineX2, double lineY2, double x, double y) {
        if(lineX1 == lineX2){
            lineX1 = lineX2 + 0.01;
        }
        if(lineY1 == lineY2){
            lineY1 = lineY2 + 0.01;
        }

        // Calculate the slope of the line
        double m1 = (lineY2 - lineY1)/(lineX2 - lineX1);
        // Calculate the slope perpendicular to this line
        double m2 = (lineX1 - lineX2)/(lineY2 - lineY1);

        double xClipedToLine = ((-m2 * x) + y + (m1 * lineX1) - lineY1)/(m1 - m2);
        double yClipedToLine = (m1 * (xClipedToLine - lineX1)) + lineY1;
        return new SSPoint(xClipedToLine,yClipedToLine);
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

    public void update() {
        localizer.update();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("x", localizer.getPoseEstimate().getX());
        packet.put("y", localizer.getPoseEstimate().getY());
        packet.put("odo heading", Math.toDegrees(localizer.getPoseEstimate().getHeading()));

//        fieldOverlay.setStroke("#3F51B5");
//        fieldOverlay.fillRect(localizer.getPoseEstimate().getX(), localizer.getPoseEstimate().getY(), 7, 7);

//        dashboard.sendTelemetryPacket(packet);
    }
}