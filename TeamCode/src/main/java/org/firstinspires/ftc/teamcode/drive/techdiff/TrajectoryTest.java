package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous
public class TrajectoryTest extends LinearOpMode {

    SampleMecanumDriveBase drive;
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();
        drive.setPoseEstimate(new Pose2d(-64, -35, 0));
        drive.followTrajectorySync(drive.trajectoryBuilder()
                .lineTo(new Vector2d(10, -40))
                .lineTo(new Vector2d(47, -48))
                .build());
        while(opModeIsActive())
            drive.update();
    }
}
