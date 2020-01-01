package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.yaml.snakeyaml.scanner.Constant;


@Autonomous
@Config
public class TrajectoryTest extends LinearOpMode {
    private static Trajectories trajectory = new Trajectories();
    private static TrajectoryGenerator tg = TrajectoryGenerator.INSTANCE;
    ElapsedTime timer = new ElapsedTime();
    private Servo arm = null;
    private Servo claw = null;
    private static double ARM_UP = 0.79;
    private static double ARM_DOWN = 0.13;
    private static double CLAW_GRAB = 0.165;
    private static double CLAW_RELEASE = 0.69;
    public static double timeout1 = 10;
    public static double timeout2 = 10;
    public static double timeout3 = 10;
    public static double timeout4 = 10;
    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
//        Glide bot = new Glide();
//        init(hardwareMap);
        waitForStart();
        armDown();
        clawRelease();
        Trajectory t = trajectory.getTrajectory(Trajectories.State.MIDDLE_STONE);
        drive.followTrajectorySync(t);
        clawGrab();
        delay(2);
        armUp();
        delay(1);
        drive.setPoseEstimate(new Pose2d(0,0,0 ));
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(2)
                        .build()
        );
        drive.turnSync(Math.toRadians(-6));
        clawRelease();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .back(75)
                .build()
        );
        armDown();
        delay(0.7);
        armUp();
        delay(0.6);
        armDown();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .strafeLeft(2)
                .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .forward(83)
                .build()
        );
//        drive.turnSync(Math.toRadians(3));
//        drive.followTrajectorySync(traj.intoStone);
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .strafeRight(2)
//                        .build()
//        );
        clawGrab();
        delay(0.6);
        armUp();
        telemetry.addData("kill the","motors");
        telemetry.update();
    }

    public void delay(double time) {
        timer.reset();
        while(opModeIsActive()) {
            if(timer.seconds() > time) break;
        }
    }


    public void setClaw(double pos) {
        pos = Range.clip(pos,0,1);
        claw.setPosition(pos);
    }

    public void setArm(double pos) {
        pos = Range.clip(pos,0,1);
        arm.setPosition(pos);
    }

    public void armUp() {
        setArm(ARM_UP);
    }

    public void armDown() {
        setArm(ARM_DOWN);
    }

    public void clawGrab() {
        setClaw(CLAW_GRAB);
    }

    public void clawRelease() {
        setClaw(CLAW_RELEASE);
    }
}