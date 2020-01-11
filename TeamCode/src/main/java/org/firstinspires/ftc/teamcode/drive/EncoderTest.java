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

//    private static double offset = 0.14;

    private static double R_ARM_STOW = 0.21 ;
    private static double R_ARM_GRAB = 0.66 ;
    private static double R_ARM_OVER = 0.58;
    private static double R_ARM_DROP = 0.35;

    public static double R_CLAW_STOW = 0.65;
    public static double R_CLAW_GRAB = 0.6;
    public static double R_CLAW_RELEASE = 0.3;

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

    // do
    public static double L_CLAW_STOW = 0.25;
    public static double L_CLAW_GRAB = 0.3;
    public static double L_CLAW_RELEASE = 0.6;

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
            telemetry.addData("y: ARM_GRAB, ROTATE_SIDE", 0);
            telemetry.addData("x: ARM_STOW, ROTATE_BACK", 0);
            telemetry.addData("default: ARM_GRAB, ROTATE_DEPOSIT", 0);
            telemetry.addData("dpad_up: CLAW_STOW", 0);
            telemetry.addData("b: CLAW_GRAB", 0);
            telemetry.addData("a: ARM_OVER", 0);

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
