package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class EncoderTest extends LinearOpMode {

    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;
    DcMotor lin;
    DcMotor rin;
    DcMotor lift1;
    DcMotor lift2;
    DistanceSensor autodist;

    @Override
    public void runOpMode() throws InterruptedException {

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lin = hardwareMap.get(DcMotor.class, "lin");
        rin = hardwareMap.get(DcMotor.class, "rin");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        autodist = hardwareMap.get(DistanceSensor.class, "autodist");

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(!isStopRequested()) {
//            lf.setPower(0.5);
//            rf.setPower(0.5);
//            lb.setPower(0.5);
//            rb.setPower(0.5);
//            rin.setPower(0.5);
//            lin.setPower(0.5);
//            lift1.setPower(0.5);
//            lift2.setPower(0.5);
            telemetry.addData("lf", lf.getCurrentPosition());
            telemetry.addData("rf", rf.getCurrentPosition());
            telemetry.addData("rb", rb.getCurrentPosition());
            telemetry.addData("lb", lb.getCurrentPosition());
            telemetry.addData("lin (strafe)", lin.getCurrentPosition());
            telemetry.addData("rin (none)", rin.getCurrentPosition());
            telemetry.addData("lift1 (right)", lift1.getCurrentPosition());
            telemetry.addData("lift2 (left)", lift2.getCurrentPosition());
            telemetry.addData("autodist", autodist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

}
