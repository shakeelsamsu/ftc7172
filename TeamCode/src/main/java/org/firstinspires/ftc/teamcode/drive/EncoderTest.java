package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class EncoderTest extends LinearOpMode {
    DcMotor lin, rin, lift2;
    public void runOpMode() {
        lin = hardwareMap.get(DcMotor.class, "lin");
        rin = hardwareMap.get(DcMotor.class, "rin");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("lin", lin.getCurrentPosition());
            telemetry.addData("rin", rin.getCurrentPosition());
            telemetry.addData("lift2", lift2.getCurrentPosition());
            telemetry.update();
        }
    }
}
