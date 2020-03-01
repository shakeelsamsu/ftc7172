package org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp
public class SSPursuitOpMode extends OpMode {
    SSGlideP glide;

    public void init() {
        glide = new SSGlideP();
        glide.init(hardwareMap);
        // check pose estimate in SSGlideP.java
    }

    public void loop() {
        ArrayList<SSCurvePoint> allPoints = new ArrayList<SSCurvePoint>();

        /*
        allPoints.add(new SSCurvePoint(0, 0, 1.0, 1.0, 16, 10, Math.toRadians(50), 1.0));
        allPoints.add(new SSCurvePoint(24, 12, 1.0, 1.0, 16, 10, Math.toRadians(50), 1.0));
        allPoints.add(new SSCurvePoint(48, 24, 1.0, 1.0, 16, 10, Math.toRadians(50), 1.0));
        */

//        allPoints.add(new SSCurvePoint(0, 0, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(6, 12, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(12, 24, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(24, 36, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(36, 24, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(48, 12, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(36, 0, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));
//        allPoints.add(new SSCurvePoint(12, -24, 1.0, 1.0, 16, 20, Math.toRadians(50), 1.0));

        allPoints.add(new SSCurvePoint(0, -48, 1.0, 0.2, 16, 20, Math.toRadians(50), 0));
        allPoints.add(new SSCurvePoint(12, 0, 1.0, 0.2, 16, 20, Math.toRadians(50), 0));
        allPoints.add(new SSCurvePoint(24, 48, 1.0, 0.2, 16, 20, Math.toRadians(50), 0));

//         glide.goToPosition(96, 0, 1.0, Math.toRadians(45), 0.3);
        glide.followCurve(allPoints, Math.toRadians(90));
        glide.displayTelemetry(telemetry);
        telemetry.update();
    }
}
