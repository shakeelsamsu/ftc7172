package org.firstinspires.ftc.teamcode.drive.techdiff.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous
@Disabled
public class TrajectoryTesting extends LinearOpMode {
    SampleMecanumDriveBase drive;
    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-38, -63, 0));
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(new Vector2d(-64, -38)).build());
    }
}
