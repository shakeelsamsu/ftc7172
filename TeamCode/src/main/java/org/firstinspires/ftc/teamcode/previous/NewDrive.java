package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
@Config
public class NewDrive extends LinearOpMode {
    boolean lastX = false;
    boolean foundation = true;
    boolean lastY = false;
    boolean gan = false;
    int currLiftLevel = 0;
    int liftCounter = 0;
    boolean lastLiftUp = false;
    boolean lastLiftDown = false;
    boolean ungrab = false;
    public static double CAP_POWER = .5;
    @Override
    public void runOpMode() {
        
        Glide bot = new Glide();
        DualPad gpad = new DualPad();
        bot.init(hardwareMap);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        
        
        waitForStart();

        while (opModeIsActive()) {
            gpad.mergePads(gamepad1,gamepad2);
            double rx = gpad.left_stick_x;
            double ry = -gpad.left_stick_y;
            double rw = -gpad.right_stick_x;
            bot.rmoveXYW(rx,ry,rw);
            bot.displayStatus(telemetry);
            // INTAKE
            double inPower = 0;
            if(gpad.right_trigger > 0.5) {
                inPower = 1;
                if (ungrab) {
                    bot.letGo();
                    ungrab = false;
                }
                bot.cancel();
            }
            if(gpad.right_stick_button) {
                inPower = -0.75;
                bot.cancel();
            }
            bot.inPower(inPower);
            bot.update();
            
            // MANUAL LIFT
            double liftPower = 0;
            if(gpad.dpad_left) {
                liftPower = -0.5; //temp
                bot.cancel();
            }
            if(gpad.dpad_right) {
                liftPower = 0.5; //temp
                bot.cancel();
            }
            bot.liftPower(liftPower);
            
            //AUTOMATIC LIFT
            
            boolean liftUp = gpad.dpad_up;
            if(liftUp && !lastLiftUp) {
                currLiftLevel = Range.clip(currLiftLevel + 1, 0, bot.MAX_HEIGHT);
                if (bot.getLiftLevel() == 0) currLiftLevel = 1;
                bot.setLiftLevel(currLiftLevel);
            }
            lastLiftUp = liftUp;
            
            boolean liftDown = gpad.dpad_down;
            if(liftDown && !lastLiftDown) {
                currLiftLevel = Range.clip(currLiftLevel - 1, 0, bot.MAX_HEIGHT);
                bot.setLiftLevel(currLiftLevel);
            }
            lastLiftDown = liftDown;
            
            boolean y = gpad.y;
            if(y && !lastY) {
                //up
                if(bot.getLiftLevel() == 0) { //changed from using liftCounter
                    currLiftLevel = Range.clip(currLiftLevel + 1, 0, bot.MAX_HEIGHT);
                    bot.setLiftLevel(currLiftLevel);
                    bot.setAuto(Glide.Auto.LIFT);
                }
                else {
                    bot.setGantry(bot.GANTRY_RETRACT);
                    ungrab = true;
                    bot.setAuto(Glide.Auto.HOME);
                }
            }
            lastY = y;
            
            
            // GANTRY
            
            // double gantryPower = 0;
            // double grabPower = 0;
            if(gpad.right_bumper) {
                bot.armDown();
                // bot.moveGantry(-.01);
                //bot.setGantry(bot.GANTRY_EXTEND);
                bot.cancel();
            }
            if(gpad.left_bumper) {
                bot.armUp();
                // bot.moveGantry(.01);
                //bot.setGantry(bot.GANTRY_RETRACT);
                bot.cancel();
            }
            
            // GRAB
            
            if (gpad.b) {
                bot.clawGrab();
                //bot.letGo();
            //   bot.moveGrab(-0.01);
                bot.cancel();
            }
            if (gpad.a) {
                bot.clawRelease();
                //bot.grabBlock();
                // bot.moveGrab(-0.005);
                bot.cancel();
            }
            
            //FOUNDATION
            boolean x = gpad.x;
            if(x && !lastX) foundation = !foundation;
            lastX = x;
            if(foundation) bot.setFoundation(0.45);
            else bot.setFoundation(0.75);
            
            // boolean y = gpad.y;
            // if(y && !lastY) gan = !gan;
            // lastY = y;
            // if(gan) bot.setGantry(bot.GANTRY_EXTEND);
            // else bot.setGantry(bot.GANTRY_RETRACT);
            
             //capstone
             double capPow = 0;
             if (gpad.left_trigger>0.5) capPow = CAP_POWER;
             bot.capstonePower(capPow);
            

        }
    }
}
