package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class EncoderTest extends LinearOpMode {
    DcMotor lin, rin, lift2,a,b,c,d;
    public void runOpMode() {
        a = hardwareMap.get(DcMotor.class, "lb");
        b = hardwareMap.get(DcMotor.class, "rb");
        c = hardwareMap.get(DcMotor.class, "lf");
        d = hardwareMap.get(DcMotor.class, "rf");
        lin = hardwareMap.get(DcMotor.class, "lin");
        rin = hardwareMap.get(DcMotor.class, "rin");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        waitForStart();
        while (opModeIsActive()) {
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
}
