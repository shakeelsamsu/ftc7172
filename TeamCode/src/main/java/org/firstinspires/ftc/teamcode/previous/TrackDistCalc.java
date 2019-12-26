package org.firstinspires.ftc.teamcode.previous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class TrackDistCalc extends LinearOpMode {

    @Override
    public void runOpMode() {
        Glide bot = new Glide();
        bot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
       // bot.driveHeading(this, .25,0,6000);
       // bot.driveHeading(this, .25,-3,11000);
        bot.setMotorPower(0,0);
        
        while (opModeIsActive()) {
            bot.displayStatus(telemetry);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
