package org.firstinspires.ftc.teamcode.drive.mecanum;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizerNewRadii;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx hub1; // 7172
    private ExpansionHubEx hub2; // 7172
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        hub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"); // 7172
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"); // 7172

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN); // 7172

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "lf");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "lb");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rb");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rf");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        leftFront.setDirection(DcMotor.Direction.REVERSE); // 7172
        leftRear.setDirection(DcMotor.Direction.REVERSE); // 7172
        rightFront.setDirection(DcMotor.Direction.FORWARD); // 7172
        rightRear.setDirection(DcMotor.Direction.FORWARD); // 7172

//        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap)); // 7172
        setLocalizer(new StandardTrackingWheelLocalizerNewRadii(hardwareMap)); // 7172
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        // 7172
        RevBulkData bulkData1 = hub1.getBulkInputData();
        RevBulkData bulkData2 = hub2.getBulkInputData();


        if (bulkData1 == null || bulkData2 == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        // 7172
        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(bulkData1.getMotorCurrentPosition(leftFront)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftRear)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(rightRear)));
        wheelPositions.add(encoderTicksToInches(bulkData1.getMotorCurrentPosition(rightFront)));
//        for (ExpansionHubMotor motor : motors) {
//            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
//        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        // 7172
        RevBulkData bulkData1 = hub1.getBulkInputData();
        RevBulkData bulkData2 = hub2.getBulkInputData();

        if (bulkData1 == null || bulkData2 == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        // 7172
        List<Double> wheelVelocities = new ArrayList<>();
        wheelVelocities.add(encoderTicksToInches(bulkData1.getMotorVelocity(leftFront)));
        wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorVelocity(leftRear)));
        wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorVelocity(rightRear)));
        wheelVelocities.add(encoderTicksToInches(bulkData1.getMotorVelocity(rightFront)));
//        for (ExpansionHubMotor motor : motors) {
//            wheelVelocities.add(encoderTicksToInches(bulkData.getMotorVelocity(motor)));
//        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
