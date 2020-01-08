package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.yaml.snakeyaml.scanner.Constant;

@Autonomous(group="test", name="DriveTest")
public class DriveTest extends LinearOpMode {

    private static ElapsedTime timer;

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        timer.reset();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        waitForStart();
        drive.setPoseEstimate(new Pose2d(-48, 0, 0));
        ConstantInterpolator interpolator = new ConstantInterpolator(0);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .lineTo(new Vector2d(48,12),interpolator)
                .build()
        );
        while (!isStopRequested()) {
            telemetry.addData("imu heading", drive.getExternalHeading());
            telemetry.update();
        }
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .lineTo(new Vector2d(-36, -32), interpolator)
//                        .build()
//        );
//        delay(5);
//
//        int n = 3;
//        while(n-- > 0) {
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .lineTo(new Vector2d(-36, -36), interpolator)
//                            .lineTo(new Vector2d(47.5, -36), interpolator)
//                            .lineTo(new Vector2d(47.5, -32), interpolator)
//                            .build()
//            );
//            delay(1);
//            drive.followTrajectorySync(
//                    drive.trajectoryBuilder()
//                            .lineTo(new Vector2d(47.5, -36), interpolator)
//                            .lineTo(new Vector2d(-36, -36), interpolator)
//                            .lineTo(new Vector2d(-36, -32), interpolator)
//                            .reverse()
//                            .build()
//            );
//            delay(1);
//        }
    }

    public void delay(double seconds) {
        timer.reset();
        while(opModeIsActive() && timer.seconds() < seconds) {

        }
    }
}
