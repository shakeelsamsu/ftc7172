package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.previous.DualPad;
import org.firstinspires.ftc.teamcode.previous.Glide;

@TeleOp

public class Drive extends LinearOpMode {

    int currLiftLevel = 0;
    int liftCounter = 0;
    boolean lastLiftUp = false;
    boolean lastLiftDown = false;
    boolean lastY = false;
    boolean lastX = false;
    boolean foundation = false;
    double alignPowerLast = 0;
    DualPad gpad;
    
    @Override
    public void runOpMode() {
        Glide bot = new Glide();
        gpad = new DualPad();
        bot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // gpad.mergePads(gamepad1,gamepad2);
            // double rx = gpad.left_stick_x;
            // double ry = -gpad.left_stick_y;
            // double rw = -gpad.right_stick_x;
            // if (gpad.left_trigger > 0.5) {
            //     rx = bot.getAlignStrafePower(alignPowerLast);
            //     ry = -0.2;
            //     alignPowerLast = rx;
            // }
            // else alignPowerLast = 0;
            // bot.rmoveXYW(rx,ry,rw);
            
            // //RESET ENCODERS
            // if (gamepad1.back) {
            //     bot.resetEncoders();
            // }

            // //LIFT
            // //manual lift does not work
            // double liftPower = 0;
            // if(gpad.dpad_left) {
            //     liftPower = -0.5; //temp
            //     bot.cancel();
            // }
            // if(gpad.dpad_right) {
            //     liftPower = 0.5; //temp
            //     bot.cancel();
            // }
            // bot.liftPower(liftPower);
            
            // boolean liftUp = gpad.dpad_up;
            // if(liftUp && !lastLiftUp) {
            //     currLiftLevel = Range.clip(currLiftLevel + 1, 0, bot.MAX_HEIGHT);
            //     if (bot.getLiftLevel() == 0) currLiftLevel = 1;
            //     bot.setLiftLevel(currLiftLevel);
            // }
            // lastLiftUp = liftUp;
            
            // boolean liftDown = gpad.dpad_down;
            // if(liftDown && !lastLiftDown) {
            //     currLiftLevel = Range.clip(currLiftLevel - 1, 0, bot.MAX_HEIGHT);
            //     bot.setLiftLevel(currLiftLevel);
            // }
            // lastLiftDown = liftDown;
            
            // boolean y = gpad.y;
            // if(y && !lastY) {
            //     //up
            //     if(bot.getLiftLevel() == 0) { //changed from using liftCounter
            //         currLiftLevel = Range.clip(currLiftLevel + 1, 0, bot.MAX_HEIGHT);
            //         bot.setLiftLevel(currLiftLevel);
            //         bot.setAuto(Glide.Auto.LIFT);
            //     }
            //     else {
            //         // bot.setAuto(Glide.Auto.HOME);
            //         bot.setGantry(2, bot.GANTRY_STOP);
            //         bot.setAuto(Glide.Auto.HOME);
            //     }
            // }
            // lastY = y;
            
            // //INTAKE
            // double inPower = 0;
            // if(gpad.right_trigger > 0.5) {
            //     inPower = 1;
            //     bot.cancel();
            // }
            // if(gpad.start) {
            //     inPower = -0.75;
            //     bot.cancel();
            // }
            // bot.inPower(inPower);
            
            // //FOUNDATION
            // boolean x = gpad.x;
            // if(x && !lastX) foundation = !foundation;
            // lastX = x;
            // if(foundation) bot.setFoundation(0.1);
            // else bot.setFoundation(0.5);
            
            // //GRABBER & GANTRY
            
            // double gantryPower = 0;
            // double grabPower = 0;
            // if(gpad.a) {
            //     grabPower = 1;
            //     bot.cancel();
            // }
            // if(gpad.b) {
            //     if(bot.getLiftLevel() != 0) 
            //         gantryPower = -0.3;
            //     grabPower = -1;
            //     // if(!y) bot.cancel();
            //     bot.cancel();
            // }
            
            // bot.setGrab(grabPower);
            
            // if(gpad.left_bumper) {
            //     gantryPower = 1; //0.4 min
            //     bot.cancel();
            // }
            // if(gpad.right_bumper) {
            //     gantryPower = -1;
            //     bot.cancel();
            // }
            // bot.setGantry(gantryPower, 0);
            
            // // int reverse = 1;
            // // if(gpad.back) {
            // //     reverse = -1;
            // //     inPower = 1;
            // // }
            // // telemetry.addData("inPower", inPower);
            // // bot.inPower(inPower, inPower * reverse);
            
            // bot.displayStatus(telemetry);            
            // bot.update();
        }
    }
}
