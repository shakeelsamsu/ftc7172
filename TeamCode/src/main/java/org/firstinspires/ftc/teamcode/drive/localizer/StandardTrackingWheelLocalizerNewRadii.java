package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class StandardTrackingWheelLocalizerNewRadii extends ThreeTrackingWheelLocalizer {
    // 7172
    public static double TICKS_PER_REV = 8192;
    public static double LATERAL_WHEEL_RADIUS = 1.145; // in
    public static double FORWARD_WHEEL_RADIUS = 1.145; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //7172
//    public static double LATERAL_DISTANCE = 12.35; // in; distance between the left and right wheels
//    public static double FORWARD_OFFSET = -6; // in; offset of the lateral wheel
//    public static double LATERAL_OFFSET = -.75; //in; offset of the lateral wheel in the y direction

    public static double LATERAL_DISTANCE = 12.7; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6; // in; offset of the lateral wheel
    public static double LATERAL_OFFSET = -0.75; //in; offset of the lateral wheel in the y direction

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizerNewRadii(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, LATERAL_OFFSET, Math.toRadians(90)) // front // 7172
        ));

        leftEncoder = hardwareMap.dcMotor.get("rin");
        rightEncoder = hardwareMap.dcMotor.get("lift2");
        frontEncoder = hardwareMap.dcMotor.get("lin");
        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static double lateralEncoderTicksToInches(int ticks) {
        return LATERAL_WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double encoderTicksToInches(int ticks) {
        return FORWARD_WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                lateralEncoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }
}
