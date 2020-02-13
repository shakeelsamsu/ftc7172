package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@TeleOp(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));

        while(!isStopRequested()) {
            telemetry.addData("IMU HEading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("odo heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            packet.put("imu heading", Math.toDegrees(drive.getExternalHeading()));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
