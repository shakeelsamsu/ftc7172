package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Disabled
@TeleOp
public class ServoDebug extends LinearOpMode {
    Servo foundation;
    public static double POS = 0.5;

    @Override
    public void runOpMode() {
        foundation = hardwareMap.get(Servo.class, "foundation");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) foundation.setPosition(POS);
            else foundation.setPosition(0.5);
        }

    }
}
