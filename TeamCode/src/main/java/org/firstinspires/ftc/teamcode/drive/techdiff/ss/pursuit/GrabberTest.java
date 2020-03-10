package org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class GrabberTest extends OpMode {
    Servo grab;

    public static double GRAB_POS = 0.55;

    public void init() {
        grab = hardwareMap.get(Servo.class, "grabber");
    }

    public void loop() {
        grab.setPosition(GRAB_POS);
    }
}
