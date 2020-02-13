package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Disabled
@Autonomous
public class NewTrajectoryBuilderTest extends LinearOpMode {

    ElapsedTime clock = new ElapsedTime();

    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();
        clock.reset();
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        Trajectory t = drive.trajectoryBuilderSlow().forward(20).build();
        followTrajectory(drive, t);
        Trajectory t2 = drive.trajectoryBuilderTest(t, t.duration() - 0.5).forward(20).build();
        followTrajectory(drive, t2);
        delay(5);
    }

    public void followTrajectory(SampleMecanumDriveBase drive, Trajectory t) {
        drive.followTrajectory(t);
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.update();
        }
    }

    public void delay(double sec) {
        double now = clock.seconds();
        while(opModeIsActive() && clock.seconds() < now + sec);

    }
}
