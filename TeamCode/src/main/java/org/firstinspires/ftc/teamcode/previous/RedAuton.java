package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


@Autonomous
@Config
public class RedAuton extends LinearOpMode {
    ElapsedTime timer;
    Glide bot;
    //UNFOLD VARS (change to public to see in dashboard)
    private static double T1 = 0.5;
    private static double T3 = 1;
    private static double T4 = 0.75;
    private static double liftPower = 0.5;
    private static double liftP2 = -0.75;

    //STRAFE and PICKUP VARS
    public static double whichStone = 0;
    public static double XD = -24;
    public static double YD = -36;
    @Override
    public void runOpMode() {
        bot = new Glide();
        timer = new ElapsedTime();
        bot.init(hardwareMap);
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        telemetry.addData("Status:", "Initialized");
        waitForStart();

        //UNFOLD ROBOT -----------------------------------------------------------------
//        bot.liftPower(liftPower);
//        bot.sleep(this,T1);
//        bot.liftPower(0);
        bot.armDown();
        bot.clawRelease();
//        bot.sleep(this,T3);
//        bot.liftPower(liftP2);
//        bot.sleep(this,T4);

        //STRAFE TO STONE and PICKUP ---------------------------------------------------
        drive.setPoseEstimate(new Pose2d(-24,-60,0));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(XD,YD))
                        .build()
        );
    }

    public void delay(double time) {
        while(opModeIsActive()) {
            if(timer.seconds() > time) break;
        }
    }
}