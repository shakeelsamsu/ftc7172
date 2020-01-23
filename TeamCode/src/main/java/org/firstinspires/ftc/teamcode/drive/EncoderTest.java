package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.techdiff.GlideConstants;
import org.openftc.revextensions2.ExpansionHubEx;

@Config
@TeleOp
public class EncoderTest extends LinearOpMode {
    DcMotor lin, rin, lift2,a,b,c,d;

//    private static double offset = 0.14;

    private static double R_ARM_STOW = GlideConstants.R_ARM_STOW;
    private static double R_ARM_GRAB = GlideConstants.R_ARM_GRAB;
    private static double R_ARM_OVER = GlideConstants.R_ARM_OVER;
    private static double R_ARM_DROP = GlideConstants.R_ARM_DROP;

    public static double R_CLAW_STOW = GlideConstants.R_CLAW_STOW;
    public static double R_CLAW_GRAB = GlideConstants.R_CLAW_GRAB;
    public static double R_CLAW_RELEASE = GlideConstants.R_CLAW_RELEASE;
    public static double R_CLAW_FOUNDATION = GlideConstants.R_CLAW_FOUNDATION;

    private static double R_ROTATE_SIDE = GlideConstants.R_ROTATE_SIDE;
    private static double R_ROTATE_DEPOSIT = GlideConstants.R_ROTATE_DEPOSIT;
    private static double R_ROTATE_BACK = GlideConstants.R_ROTATE_BACK;
    //
    public static double L_ARM_STOW = GlideConstants.L_ARM_STOW;
    public static double L_ARM_GRAB = GlideConstants.L_ARM_GRAB;
    public static double L_ARM_OVER = GlideConstants.L_ARM_OVER;
    public static double L_ARM_DROP = GlideConstants.L_ARM_DROP;

    // done
    public static double L_CLAW_STOW = GlideConstants.L_CLAW_STOW;
    public static double L_CLAW_GRAB = GlideConstants.L_CLAW_GRAB;
    public static double L_CLAW_RELEASE = GlideConstants.L_CLAW_RELEASE;
    public static double L_CLAW_FOUNDATION= GlideConstants.L_CLAW_FOUNDATION;

    public static double L_ROTATE_SIDE = GlideConstants.L_ROTATE_SIDE;
    public static double L_ROTATE_DEPOSIT = GlideConstants.L_ROTATE_DEPOSIT;
    public static double L_ROTATE_BACK = GlideConstants.L_ROTATE_BACK;
    private Servo rarm;
    private Servo rrotate;
    private Servo rclaw;
    private Servo larm;
    private Servo lclaw;
    private Servo lrotate;
    private ExpansionHubEx hub1;
    private ExpansionHubEx hub2;


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
        hub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"); // 7172
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"); // 7172

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {LsetArm(L_ARM_STOW);RsetArm(R_ARM_STOW);}
            else if (gamepad1.y) {LsetArm(L_ARM_GRAB);RsetArm(R_ARM_GRAB);}
            else if (gamepad1.a) {LsetArm(L_ARM_OVER);RsetArm(R_ARM_OVER);}
            else {LsetArm(L_ARM_DROP);RsetArm(R_ARM_DROP);}

            if(gamepad1.x) {LsetClaw(L_CLAW_FOUNDATION); RsetClaw(R_CLAW_FOUNDATION);}
            else if (gamepad1.b) {LsetClaw(L_CLAW_GRAB);RsetClaw(R_CLAW_GRAB);}
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
            telemetry.addData("hub1 servo current", hub1.getServoBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("hub2 servo current", hub2.getServoBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
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
