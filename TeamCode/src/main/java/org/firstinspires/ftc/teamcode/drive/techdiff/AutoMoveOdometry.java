package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@TeleOp
public class AutoMoveOdometry extends LinearOpMode {

    private SampleMecanumDriveBase drive;
    private double dstHeading = 0;
    private double dstX = 0;
    private double dstY = 0;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            double rx = gamepad1.left_stick_x;
            double ry = -gamepad1.left_stick_y;
            double rw = -gamepad1.right_stick_x;
            double currX= drive.getPoseEstimate().getX();
            double currY = drive.getPoseEstimate().getY();
            double currHeading = drive.getPoseEstimate().getHeading();
            if(gamepad1.a) {
                dstX = currX;
                dstY = currY;
                dstHeading = currHeading;
            }
            if(gamepad1.left_trigger > 0.25) {
//                if(Math.abs(currHeading - heading) > Math.toRadians(1)) {
//                if(rx == 0 && ry == 0)
//                    rw = Math.toDegrees(dstHeading- currHeading) * 0.015;
//                else
//                    rx = (dstX - currX) * 0.02;
//                    ry = (dstY - currY) * 0.02;
//                    rw = Math.toDegrees(dstHeading - currHeading) * 0.01;
//                }
                drive.followTrajectorySync(
                        drive.trajectoryBuilder()
                                .reverse()
                                .splineTo(new Pose2d(dstX, dstY), new ConstantInterpolator(Math.toRadians(dstHeading)))
                                .build()
                );
            }
            else {
                drive.setDrivePower(new Pose2d(ry, rx, rw));
            }
            drive.update();

            telemetry.addData("dst x", dstX);
            telemetry.addData("odo x", drive.getPoseEstimate().getX());

            telemetry.addData("dst y", dstY);
            telemetry.addData("odo y", drive.getPoseEstimate().getY());

            telemetry.addData("dst heading", dstHeading);
            telemetry.addData("ext heading", drive.getExternalHeading());
            telemetry.addData("odo heading", drive.getPoseEstimate().getHeading());

            telemetry.update();
        }
    }
}
