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
    }

    public void loop() {
        ArrayList<SSCurvePoint> allPoints = new ArrayList<SSCurvePoint>();
        allPoints.add(new SSCurvePoint(0, 0, 1.0, 0.3, 16, Math.toRadians(50), 1.0));
        allPoints.add(new SSCurvePoint(24, 0, 1.0, 0.3, 16, Math.toRadians(50), 1.0));
        allPoints.add(new SSCurvePoint(48, 48, 1.0, 0.3, 16, Math.toRadians(50), 1.0));
        allPoints.add(new SSCurvePoint(48, 24, 1.0, 0.3, 16, Math.toRadians(30), 1.0));
        allPoints.add(new SSCurvePoint(24, 12, 1.0, 0.3, 16, Math.toRadians(30), 1.0));
        allPoints.add(new SSCurvePoint(0, 0, 1.0, 0.3, 16, Math.toRadians(30), 1.0));
        // glide.goToPosition(30, 30, 1.0, Math.toRadians(90), 0.3);
        glide.followCurve(allPoints, Math.toRadians(90));
        glide.displayTelemetry(telemetry);
        telemetry.update();
    }
}
