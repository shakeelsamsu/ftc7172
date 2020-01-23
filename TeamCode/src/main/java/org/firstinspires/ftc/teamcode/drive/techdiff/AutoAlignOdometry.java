package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
@Disabled
@TeleOp
public class AutoAlignOdometry extends LinearOpMode {

    private SampleMecanumDriveBase drive;
    private double heading = 0;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            double rx = gamepad1.left_stick_x;
            double ry = -gamepad1.left_stick_y;
            double rw = -gamepad1.right_stick_x;
            double currHeading = drive.getPoseEstimate().getHeading();
            if(gamepad1.a)
                heading = currHeading;
            if(gamepad1.left_trigger > 0.25) {
                if(Math.abs(currHeading - heading) > Math.toRadians(1)) {
                    if(rx == 0 && ry == 0)
                        rw = Math.toDegrees(heading - currHeading) * 0.015;
                    else
                        rw = Math.toDegrees(heading - currHeading) * 0.01;
                }
            }
            drive.setDrivePower(new Pose2d(ry, rx, rw));
            drive.update();

            telemetry.addData("dst heading", heading);
            telemetry.addData("ext heading", drive.getExternalHeading());
            telemetry.addData("odo heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
