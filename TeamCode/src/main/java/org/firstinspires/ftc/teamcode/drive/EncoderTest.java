package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp
public class EncoderTest extends LinearOpMode {
    DcMotor lin, rin, lift2,a,b,c,d;
    private static double R_ARM_STOW = 0.21;
    private static double R_ARM_GRAB = 0.66;
    private static double R_ARM_OVER = 0.58;
    private static double R_ARM_DROP = 0.35;

    public static double R_CLAW_STOW = 0.85;
    public static double R_CLAW_GRAB = 0.7;
    public static double R_CLAW_RELEASE = 0.17;

    private static double R_ROTATE_SIDE = 0.47;
    private static double R_ROTATE_DEPOSIT = 0.105;
    private static double R_ROTATE_BACK = 0;

    private static double FOUNDATION_GRAB = 0.75;
    private static double FOUNDATION_RELEASE = 0.5;

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
    private Servo rarm;
    private Servo rrotate;
    private Servo rclaw;
    private Servo larm;
    private Servo lclaw;
    private Servo lrotate;
    public void runOpMode() {
        a = hardwareMap.get(DcMotor.class, "lb");
        b = hardwareMap.get(DcMotor.class, "rb");
        c = hardwareMap.get(DcMotor.class, "lf");
        d = hardwareMap.get(DcMotor.class, "rf");
        lin = hardwareMap.get(DcMotor.class, "lin");
        rin = hardwareMap.get(DcMotor.class, "rin");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        rarm = hardwareMap.get(Servo.class, "rarm");
        rrotate = hardwareMap.get(Servo.class, "rrotate");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        larm = hardwareMap.get(Servo.class, "larm");
        lrotate = hardwareMap.get(Servo.class, "lrotate");
        lclaw = hardwareMap.get(Servo.class, "lclaw");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {LsetArm(L_ARM_STOW);RsetArm(R_ARM_STOW);}
            else if (gamepad1.y) {LsetArm(L_ARM_GRAB);RsetArm(R_ARM_GRAB);}
            else if (gamepad1.a) {LsetArm(L_ARM_OVER);RsetArm(R_ARM_OVER);}
            else {LsetArm(L_ARM_DROP);RsetArm(R_ARM_DROP);}

            if (gamepad1.b) {LsetClaw(L_CLAW_GRAB);RsetClaw(R_CLAW_GRAB);}
            else if (gamepad1.dpad_up) {LsetClaw(L_CLAW_STOW);RsetClaw(R_CLAW_STOW);}
            else {LsetClaw(L_CLAW_RELEASE);RsetClaw(R_CLAW_RELEASE);}

            if (gamepad2.x) {LsetRotate(L_ROTATE_BACK);RsetRotate(R_ROTATE_BACK);}
            else if (gamepad2.y) {LsetRotate(L_ROTATE_SIDE);RsetRotate(R_ROTATE_SIDE);}
            else {LsetRotate(L_ROTATE_DEPOSIT);RsetRotate(R_ROTATE_DEPOSIT);}
            telemetry.addData("lin", lin.getCurrentPosition());
            telemetry.addData("rin", rin.getCurrentPosition());
            telemetry.addData("lift2", lift2.getCurrentPosition());
            telemetry.addData("rb", b.getCurrentPosition());
            telemetry.addData("lb", a.getCurrentPosition());
            telemetry.addData("lf", c.getCurrentPosition());
            telemetry.addData("rf", d.getCurrentPosition());
            telemetry.update();
        }
    }
    public void RsetArm(double p) {
        rarm.setPosition(p);
    }
    public void RsetClaw(double p) {
        rclaw.setPosition(p);
    }
    public void RsetRotate(double p) {
        rrotate.setPosition(p);
    }


    public void LsetArm(double p) {
        larm.setPosition(p);
    }
    public void LsetClaw(double p) {
        lclaw.setPosition(p);
    }
    public void LsetRotate(double p) {
        lrotate.setPosition(p);
    }

}