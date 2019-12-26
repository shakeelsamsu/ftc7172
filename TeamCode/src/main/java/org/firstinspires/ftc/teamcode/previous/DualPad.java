package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DualPad {
    public boolean shift1;
    public boolean shift2;

    //buttons
    public boolean a;
    public boolean aShift;

    public boolean b;
    public boolean bShift;

    public boolean x;
    public boolean xShift;

    public boolean y;
    public boolean yShift;

    public boolean left_bumper;
    public boolean left_bumperShift;

    public boolean right_bumper;
    public boolean right_bumperShift;

    //dpad
    public boolean dpad_up;
    public boolean dpad_upShift;

    public boolean dpad_left;
    public boolean dpad_leftShift;

    public boolean dpad_down;
    public boolean dpad_downShift;

    public boolean dpad_right;
    public boolean dpad_rightShift;

    public boolean start;
    public boolean startShift;

    public boolean back;
    public boolean backShift;

    public boolean guide;
    public boolean guideShift;
    
    public boolean right_stick_button;
    public boolean right_stick_buttonShift;
    
    //axis
    double left_trigger;
    double right_trigger;
    double left_stick_x;
    double left_stick_y;
    double right_stick_x;
    double right_stick_y;

    public void mergePads(Gamepad gpad1, Gamepad gpad2 ){
        shift1 = gpad1.back;
        shift2 = gpad2.back;

        left_stick_x = gpad1.left_stick_x;
        //if (left_stick_x == 0) left_stick_x = gpad2.left_stick_x;

        left_stick_y = gpad1.left_stick_y;
        //if (left_stick_y == 0) left_stick_y = gpad2.left_stick_y;

        right_stick_x = gpad1.right_stick_x;
        //if (right_stick_x == 0) right_stick_x = gpad2.right_stick_x;

        right_stick_y = gpad1.right_stick_y;
        //if (right_stick_y == 0) right_stick_y = gpad2.right_stick_y;

        left_trigger = gpad1.left_trigger;
        if (left_trigger == 0) left_trigger = gpad2.left_trigger;

        right_trigger = gpad1.right_trigger;
        if (right_trigger == 0) right_trigger = gpad2.right_trigger;

        left_bumper = (!shift1 && gpad1.left_bumper) || (!shift2 && gpad2.left_bumper);
        left_bumperShift = (shift1 && gpad1.left_bumper) || (shift2 && gpad2.left_bumper);

        right_bumper = (!shift1 && gpad1.right_bumper) || (!shift2 && gpad2.right_bumper);
        right_bumperShift = (shift1 && gpad1.right_bumper) || (shift2 && gpad2.right_bumper);

        a = (!shift1 && gpad1.a) || (!shift2 && gpad2.a);
        aShift = (shift1 && gpad1.a) || (shift2 && gpad2.a);

        b = (!shift1 && gpad1.b) || (!shift2 && gpad2.b);
        bShift = (shift1 && gpad1.b) || (shift2 && gpad2.b);

        x = (!shift1 && gpad1.x) || (!shift2 && gpad2.x);
        xShift = (shift1 && gpad1.x) || (shift2 && gpad2.x);

        y = (!shift1 && gpad1.y) || (!shift2 && gpad2.y);
        yShift = (shift1 && gpad1.y) || (shift2 && gpad2.y);

        dpad_up = (!shift1 && gpad1.dpad_up) || (!shift2 && gpad2.dpad_up);
        dpad_upShift = (shift1 && gpad1.dpad_up) || (shift2 && gpad2.dpad_up);

        dpad_down = (!shift1 && gpad1.dpad_down) || (!shift2 && gpad2.dpad_down);
        dpad_downShift = (shift1 && gpad1.dpad_down) || (shift2 && gpad2.dpad_down);

        dpad_left = (!shift1 && gpad1.dpad_left) || (!shift2 && gpad2.dpad_left);
        dpad_leftShift = (shift1 && gpad1.dpad_left) || (shift2 && gpad2.dpad_left);

        dpad_right = (!shift1 && gpad1.dpad_right) || (!shift2 && gpad2.dpad_right);
        dpad_rightShift = (shift1 && gpad1.dpad_right) || (shift2 && gpad2.dpad_right);

        start = (!shift1 && gpad1.start) || (!shift2 && gpad2.start);
        startShift = (shift1 && gpad1.start) || (shift2 && gpad2.start);

        back = (!shift1 && gpad1.back) || (!shift2 && gpad2.back);
        backShift = (shift1 && gpad1.back) || (shift2 && gpad2.back);

        guide = (!shift1 && gpad1.guide) || (!shift2 && gpad2.guide);
        guideShift = (shift1 && gpad1.guide) || (shift2 && gpad2.guide);
        
        right_stick_button = (!shift1 && gpad1.right_stick_button) || (!shift2 && gpad2.right_stick_button);
        right_stick_buttonShift = (shift1 && gpad1.right_stick_button) || (shift2 && gpad2.right_stick_button);
        
    }
}