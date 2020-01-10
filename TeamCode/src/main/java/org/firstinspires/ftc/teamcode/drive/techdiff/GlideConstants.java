package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GlideConstants {

    public static double RAMPING_ERROR = 5000;

    public static int[] liftLevels = {0,7000,7000,11100,14500,19100,23000,27700,32000,35400,40100,44900,49400,53400,59700};
    final static int MAX_HEIGHT = 14;

    public final double BAR_FORWARD = 0.23;    // when bar is holding a stone
    public final double BAR_BACK = 0.75; // when bar points in robot's forward direction, when it is idle
    public static double GRAB_POS = 0.19;
    public static double LETGO_POS = 0.27;

    public static double R_ARM_STOW = 0.21 ;
    public static double R_ARM_GRAB = 0.66 ;
    public static double R_ARM_OVER = 0.58;
    public static double R_ARM_DROP = 0.35;

    public static double R_CLAW_STOW = 0.99;
    public static double R_CLAW_GRAB = 0.84;
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
    public static double L_CLAW_STOW = 0.09;
    public static double L_CLAW_GRAB = 0.2;
    public static double L_CLAW_RELEASE = 0.75;

    public static double L_ROTATE_SIDE = 0.165;
    public static double L_ROTATE_DEPOSIT = 0.53;
    public static double L_ROTATE_BACK = 0.64;

    final double STONE_DIST_BAR = 1.6;
    final double STONE_DIST_GRAB = 0.3;
    final double GRAB_WAIT = 0.75;
    boolean seen = false;

    public double GANTRY_RETRACT = 0.94;
    public double GANTRY_CAP = 0.55;
    public double GANTRY_EXTEND = 0.1;
    public double gantryPosition = GANTRY_RETRACT;
    public double gantryLock = 0;
    public double GANTRY_EXTEND_DELAY = 1.2;

    public final double TRACK_TICKS = 2332;
    public final double TICKS_PER_INCH = 1142.1815; //prev 190.a0
    public final int DISPLAY_ODOMETRY = 0x0001;
    public final int DISPLAY_IMU = 0x0002;
    public final int DISPLAY_LIFT = 0x0004;
    public final int DISPLAY_CALCTRACK = 0x80000000;
    public final int DISPLAY_ALL = 0xffffffff;
    public final int DISPLAY_GANTRY = 0x0008;
    public final int DISPLAY_GRAB = 0x0010;
    public final int DISPLAY_ALIGN = 0x0020;
    public final int DISPLAY_GENERAL = 0x0040;
    public final int DISPLAY_INTAKE = 0x0080;

    public final int LEVEL_DEAD_ZONE = 100;

    final double GRAB_TIME = 0.95;

    public static enum Auto {IDLE, LIFT, HOME, GRAB, SCORE};
    Auto autoState = Auto.IDLE;

    public double ALIGN_STRAFE_GAIN = 0.6;

}
