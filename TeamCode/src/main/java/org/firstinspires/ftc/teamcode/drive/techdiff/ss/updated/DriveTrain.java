package org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.MovementVars.movement_y;
import static org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated.MovementVars.movement_turn;

public class DriveTrain {
    public DcMotorEx rf;
    public DcMotorEx rb;
    public DcMotorEx lb;
    public DcMotorEx lf;

    public DriveTrain(DcMotorEx rf, DcMotorEx rb, DcMotorEx lf, DcMotorEx lb) {
        this.rf = rf;
        this.rb = rb;
        this.lf = lf;
        this.lb = lb;
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        rf.setPower(movement_y - movement_x + movement_turn);
        rb.setPower(movement_y + movement_x + movement_turn);
        lf.setPower(movement_y + movement_x - movement_turn);
        lb.setPower(movement_y - movement_x - movement_turn);
    }
}
