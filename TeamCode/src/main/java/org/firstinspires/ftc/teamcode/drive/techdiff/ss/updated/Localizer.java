package org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

public class Localizer {
    ThreeTrackingWheelLocalizer localizer;

    public static double world_x;
    public static double world_y;
    public static double world_heading;

    public Localizer(ThreeTrackingWheelLocalizer localizer) {
        this.localizer = localizer;
    }

    public void update() {
        localizer.update();
        world_x = localizer.getPoseEstimate().getX();
        world_y = localizer.getPoseEstimate().getY();
        world_heading = localizer.getPoseEstimate().getHeading();
    }

    public void setPoseEstimate(Pose2d pose2d) {
        localizer.setPoseEstimate(pose2d);
    }
}
