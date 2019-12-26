/*
Copyright 1969 FIRST Tech Challenge Team 7172

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class PmEncTest extends LinearOpMode {

    double trackTicks = 3000.0;
    
    DcMotor lf;
    DcMotor lb;
    DcMotor rf;
    DcMotor rb;
    
    BNO055IMU imu;
    
    int lfticks0;
    int rfticks0;
    double heading;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfticks0 = lf.getCurrentPosition();
        rfticks0 = rf.getCurrentPosition();

        telemetry.addData("Status", "Running");
        telemetry.update();

        driveHeading(0.25, 0, 12000);
        driveHeading(0.25, 3.14159, 17000);
        Orientation imuangles = imu.getAngularOrientation();
        telemetry.addData("heading", heading);
        telemetry.addData("imu", imuangles.firstAngle);
        telemetry.addData("diff", heading+imuangles.firstAngle);
        telemetry.update();
        driveStop();
        while (opModeIsActive()) { }
    }
    
    public void updatePosition() {
        trackTicks = 2370.0;
        int lfticks1 = lf.getCurrentPosition();
        int rfticks1 = rf.getCurrentPosition();
        int deltaTicks = (rfticks1-rfticks0) - (lfticks1-lfticks0);
        heading = deltaTicks / trackTicks;
        // lfticks0 = lfticks1;
        // rfticks0 = rfticks1;
    }

    public void driveHeading(double power, double targetHeading, int distTicks) {
        int lfstart = lf.getCurrentPosition();
        int rfstart = rf.getCurrentPosition();
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        while (opModeIsActive()) {
            updatePosition();
            int lfcurr = lf.getCurrentPosition();
            int rfcurr = rf.getCurrentPosition();
            if ((Math.abs(lfcurr-lfstart) + Math.abs(rfcurr-rfstart))/2 > distTicks) break;
            double error = targetHeading - heading;
            double kP = 1.5;
            double lpower = Range.clip(power - error * kP, 0.0, 0.8);
            double rpower = Range.clip(power + error * kP, 0.0, 0.8);
            lf.setPower(lpower); lb.setPower(lpower);
            rf.setPower(rpower); rb.setPower(rpower);
        }
    }

    

    public void driveAngle(double lpower, double rpower, double stopAngle) {
        lf.setPower(lpower); lb.setPower(lpower);
        rf.setPower(rpower); rb.setPower(rpower);
        int lfticks0 = lf.getCurrentPosition();
        int rfticks0 = rf.getCurrentPosition();
        double angle = 0;
        while (opModeIsActive()) {
            int lfticks1 = lf.getCurrentPosition();
            int rfticks1 = rf.getCurrentPosition();
            int deltaTicks = (rfticks1-rfticks0) - (lfticks1-lfticks0);
            angle += deltaTicks / trackTicks;
            
           if (Math.abs(angle) > stopAngle) break;
            lfticks0 = lfticks1;
            rfticks0 = rfticks1;
        }
    }

    public void driveFwd(double power, int distTicks, double timeout) {
        int lfticks0 = lf.getCurrentPosition();
        int rfticks0 = rf.getCurrentPosition();
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
           int lfticks1 = lf.getCurrentPosition();
           int rfticks1 = rf.getCurrentPosition();
           if (Math.abs(lfticks1 - lfticks0) > distTicks) break;
           lf.setPower(power); lb.setPower(power);
           rf.setPower(power); rb.setPower(power);
           telemetry.addData("lf.ticks1", lfticks1);
           telemetry.addData("rf.ticks1", rfticks1);
           telemetry.update();
        }
    }
    
    public void driveStop() {
        lf.setPower(0); lb.setPower(0);
        rf.setPower(0); rb.setPower(0);
    }
    
}
