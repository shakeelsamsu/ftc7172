package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    public static double a = 0;
    @Override
    public void runOpMode() {
        bot = new Glide();
        timer = new ElapsedTime();
        bot.init(hardwareMap);
        telemetry.addData("Status:", "Initialized");
        waitForStart();

        //UNFOLD ROBOT -----------------------------------------------------------------
        bot.liftPower(liftPower);
        bot.sleep(this,T1);
        bot.liftPower(0);
        bot.armDown();
        bot.clawRelease();
        bot.sleep(this,T3);
        bot.liftPower(liftP2);
        bot.sleep(this,T4);

        //STRAFE TO STONE and PICKUP ---------------------------------------------------

    }

    public void delay(double time) {
        while(opModeIsActive()) {
            if(timer.seconds() > time) break;
        }
    }
}