package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Disabled
@Config
@Autonomous
public class FoundationTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    public static double FOUNDATION_GRAB = 0.76;
    public static double FOUNDATION_RELEASE = 0.4;

    private Servo foundation;

    private SampleMecanumDriveBase drive;

    public void runOpMode() {
        timer = new ElapsedTime();
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        foundation = hardwareMap.get(Servo.class, "foundation");

        waitForStart();
//        while(opModeIsActive()) {
//            if (gamepad1.x) setFoundation(FOUNDATION_GRAB);
//            else setFoundation(FOUNDATION_RELEASE);
//        }

        drive.setPoseEstimate(new Pose2d(55, 36, 0));

        setFoundation(FOUNDATION_RELEASE);
        drive.turnSync(Math.toRadians(90));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(6)
                        .build()
        );
        setFoundation(FOUNDATION_GRAB);
        delay(0.7);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30,60,Math.toRadians(180)))
                        .build()
        );
        setFoundation(FOUNDATION_RELEASE);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(20).build()
        );


    }

    private void setFoundation(double pos) {
        foundation.setPosition(pos);
    }
    public void delay(double time) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {}
    }


}
