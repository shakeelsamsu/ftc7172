package org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;

public abstract class Auto extends OpMode {

    DriveTrain driveTrain;
    Localizer localizer;
    DcMotorEx rf, rb, lf, lb;
//    FtcDashboard dashboard;

    public void init() {
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        driveTrain = new DriveTrain(rf, rf, lf, lb);

        localizer = new Localizer(new StandardTrackingWheelLocalizer(hardwareMap));
//        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
//        dashboard.setTelemetryTransmissionInterval(25);
    }

//    public void init_loop() {
//        update();
//    }

    public void loop() {
        update();
    }

    public void update() {
        localizer.update();
        driveTrain.update();
        displayTelemetry();
    }

    public void displayTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();


        //        FtcDashboard dashboard = FtcDashboard.getInstance();
//
//        dashboard.sendTelemetryPacket(packet);
    }
}
