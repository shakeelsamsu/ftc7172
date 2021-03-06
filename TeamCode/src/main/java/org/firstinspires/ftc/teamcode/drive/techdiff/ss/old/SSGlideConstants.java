package org.firstinspires.ftc.teamcode.drive.techdiff.ss.old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
Constants class for Glide. If values are changed in Android Studio/OnBot Java,
they must be updated in the other editor as well.
*/
public class SSGlideConstants {

    public static double RAMPING_ERROR = 5000;

    //                                    1    2      3     4    5     6     7     8    9     10    11      12   13    14
    public static int[] LIFT_LEVELS = {0,7700,7700,11800,15200,19800,24000,28800,31900,35900,41000,45300,49900,53700,59800};
    final static int MAX_HEIGHT = 14;
    final static int LIFT_EXTEND_POS = 3190;

    public final double BAR_FORWARD = 0.23;    // when bar is holding a stone
    public final double BAR_BACK = 0.75; // when bar points in robot's forward direction, when it is idle
    public static double GRAB_POS = 0.275;
    public static double LETGO_POS = 0.39;
    // public static double GRAB_POS = 0;
    // public static double LETGO_POS = 1;

    public static double R_ARM_STOW = 0.21 ;
    public static double R_ARM_GRAB = 0.66 ;
    public static double R_ARM_OVER = 0.58;
    public static double R_ARM_DROP = 0.35;

    public static double R_CLAW_STOW = 0.75;
    public static double R_CLAW_GRAB = 0.7;
    public static double R_CLAW_RELEASE = 0.31;

    public static double R_ROTATE_SIDE = 0.47;
    public static double R_ROTATE_DEPOSIT = 0.105;
    public static double R_ROTATE_BACK = 0;

    public static double FOUNDATION_GRAB = 0.75;
    public static double FOUNDATION_RELEASE = 0.5;

    //
    public static double L_ARM_STOW = 0.76;
    public static double L_ARM_GRAB = 0.3;
    public static double L_ARM_OVER = 0.38;
    public static double L_ARM_DROP = 0.62;

    // done
    public static double L_CLAW_STOW = 0.2;
    public static double L_CLAW_GRAB = 0.25;
    public static double L_CLAW_RELEASE = 0.75;

    public static double L_ROTATE_SIDE = 0.165;
    public static double L_ROTATE_DEPOSIT = 0.53;
    public static double L_ROTATE_BACK = 0.64;

    final static double STONE_DIST_BAR = 1.6;
    final static double STONE_DIST_GRAB = 0.3;
    final static double GRAB_WAIT = 0.75;
    boolean seen = false;
    
    final static double CAP_POS = 0;
    final static double UNCAP_POS = 1;
    final static double CAP_IDLE = 0.5;

    public static double GANTRY_RANGE = 0.88;
    public static double GANTRY_CAP = 0.52;
    public static double GANTRY_EXTEND = 0.08;
    public static double GANTRY_RETRACT = GANTRY_EXTEND + GANTRY_RANGE;
    public static double gantryPosition = GANTRY_RETRACT;
    public static double gantryLock = 0;
    public static double GANTRY_EXTEND_DELAY = 0.5;

    public static final double TRACK_TICKS = 2332;
    public static final double TICKS_PER_INCH = 1142.1815; //prev 190.a0
    public static final int DISPLAY_ODOMETRY = 0x0001;
    public static final int DISPLAY_IMU = 0x0002;
    public static final int DISPLAY_LIFT = 0x0004;
    public static final int DISPLAY_CALCTRACK = 0x80000000;
    public static final int DISPLAY_ALL = 0xffffffff;
    public static final int DISPLAY_GANTRY = 0x0008;
    public static final int DISPLAY_GRAB = 0x0010;
    public static final int DISPLAY_ALIGN = 0x0020;
    public static final int DISPLAY_GENERAL = 0x0040;
    public static final int DISPLAY_INTAKE = 0x0080;

    public static final int LEVEL_DEAD_ZONE = 250;

    final static double GRAB_TIME = 0.95;

    public static enum Auto {IDLE, LIFT, HOME, GRAB, SCORE};
    Auto autoState = Auto.IDLE;

    public static double ALIGN_STRAFE_GAIN = 0.6;

}
