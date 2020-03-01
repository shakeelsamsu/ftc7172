package org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.SSCurvePoint;
//import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.hotpursuit.GlidePursuit.follow

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.GlidePursuit.followCurve;

@TeleOp
public class TestOp extends Auto {
    public void init() {
        super.init();

    }

    public void loop() {
        super.loop();

        ArrayList<SSCurvePoint> allPoints = new ArrayList<SSCurvePoint>();


//        allPoints.add(new SSCurvePoint(0, 0, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(6, 12, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(12, 24, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(24, 36, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(36, 24, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(48, 12, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(36, 0, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(12, -24, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));

//        allPoints.add(new SSCurvePoint(0, -48, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(12, 0, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(24, 48, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));

        allPoints.add(new SSCurvePoint(0, 0, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
        allPoints.add(new SSCurvePoint(24, 0, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
        allPoints.add(new SSCurvePoint(48, 0, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
        allPoints.add(new SSCurvePoint(72, 0,  1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));

        followCurve(allPoints, Math.toRadians(90));
    }
}
