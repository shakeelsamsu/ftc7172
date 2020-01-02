package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.util.List;


@Autonomous
@Config
public class TrajectoryTest extends LinearOpMode {
//    private static Trajectories traj = new Trajectories();
//    private static TrajectoryGenerator tg = TrajectoryGenerator.INSTANCE;
    ElapsedTime timer = new ElapsedTime();
    private Servo arm = null;
    private Servo claw = null;
    public static double ARM_UP = 0.79;
    public static double ARM_DOWN = 0.18;
    public static double CLAW_GRAB = 0.12;
    public static double CLAW_RELEASE = 0.69;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AROJMNH/////AAABmfj2l+LOQ01wteJSaDwD5yVh+yKqERKv2v+nfmUTuA9A3zwyrXAkEUuDSjKPZ5MUfmz0hP9KASiO3tvdoq2eHNAZDhIx8DyW6RDu6FbrGCgO8orfeVL+Ya7z5tPYSH++sgyHKN/ED2DLd2pM7tdSOZZepRxNdegqJDE6C1t6dzXMSbOcfCSuWqTmRQYSoO3VxeszFfrX80jm31A3t5m2KOPq0xUuKKHStEz72JW+JEfaRMKGcrszGCilBowD1sGtwcCzshXCBXsTbK88En//xB8EHjIYpA6s/awDLj3/RDo6HsrO9T2iVtpIHl8Q4325gunwdek4+VvDc9ST5Jl/1UJrD0CXpTPW0kkDwSPSCGcO";

    Frame frame = null;
    Image image = null;

    boolean lastA = false;

    private int stoneLeft = 0;
    private int stoneRight = 0;
    private int stoneBottom = 0;
    private int stoneTop = 0;
    private double stoneHeading = 0;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    int stones = 2;
    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
//        Glide bot = new Glide();
//        init(hardwareMap);
//        initVuforia();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
//        int pos = 1;
//        while (!opModeIsActive()) {
//            if (getSkystone()) {
//                if (stoneHeading < -10) pos = 1;
//                else if (stoneHeading > 10) pos = 3;
//                else pos = 2;
//            }
//            if (gamepad1.y) stones = 2;
//            if (gamepad1.a) stones = 1;
//            telemetry.addData("pos",pos);
//            telemetry.addData("angle estimate", stoneHeading);
//            telemetry.addData("stones", stones);
//            try {
//                frame = vuforia.getFrameQueue().take();
//            }
//            catch (InterruptedException e) {
//
//            }
//            if (tfod != null) {
//                tfod.activate();
//            }
//            if(frame != null) {
//                for(int i = 0; i < frame.getNumImages(); i++) {
//                    image = frame.getImage(0);
//                    // telemetry.log().add("Picture type = %d %d x %d",image.getFormat(),image.getWidth(),image.getHeight());
//                    // telemetry.log().add("Num Bytes = %d",image.getPixels().remaining());
//                    ByteBuffer pixels = image.getPixels();
//                    byte[] pixelArray = new byte[pixels.remaining()];
//                    pixels.get(pixelArray, 0, pixelArray.length);
//                    telemetry.addData("pixelArray", (pixelArray[0] & 0xFF));
//                    telemetry.addData("stoneLeft", stoneLeft);
//                    telemetry.addData("stoneRight", stoneRight);
//                    boolean a = gamepad1.a;
//                    if(a) {
//                        int[] skyStoneArr = findSkyStone2(pixelArray, stoneLeft, stoneRight, stoneBottom, stoneTop);
//                        telemetry.addData("findSkyStone", skyStoneArr[0] + " " + skyStoneArr[1]);
//                        int position = 0;
//                        if(skyStoneArr[3] < 250) {
//                            position = 1;
//                        }
//                        else if(skyStoneArr[3] < 450) {
//                            position = 2;
//                        }
//                        else if(skyStoneArr[3] < 650) {
//                            position = 3;
//                        }
//                        telemetry.addData("position", position);
//                    }
//
//                    lastA = a;
//                    // telemetry.addData("pixelArray length", pixelArray.length);
//                    telemetry.update();
//                }
//            }
//            telemetry.update();
//        }
//        tfod.shutdown();
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }

        waitForStart();
//        drive.setPoseEstimate(new Pose2d(0,0,0));
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .splineTo(new Pose2d(48,0,0))
//                .build()
//        );
//        sleep(2000);
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .splineTo(new Pose2d(0,0,0))
//                .build()
//        );
        drive.setPoseEstimate(new Pose2d(-36,-63,Math.PI));
        ConstantInterpolator interp = new ConstantInterpolator(Math.PI);


        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .lineTo(new Vector2d(-36,-32),interp)
                .build()
        );
        strafeAndGrab(drive);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(47.5, drive.getPoseEstimate().getY(), Math.PI))
                .build()
        );
        launchStone(drive);
        //drive.turnSync(drive.lastHeadingError);
//        armDown();
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .strafeLeft(6)
//                .build()
//        );

        delay(10);
//        armDown();
//        clawRelease();
//        drive.setPoseEstimate(new Pose2d(-36,-63,Math.PI));
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .strafeTo(new Vector2d(-38, -30))
//                        .build()
//        );
//        clawGrab();
//        delay(.65);         // pickup 1
//        armUp();
////        drive.followTrajectorySync(
////                drive.trajectoryBuilder()
////                .strafeLeft(4)
////                .build()
//        delay(0.6);
//        clawRelease();
//        // head to foundation 1
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .reverse()
//                        .splineTo(new Pose2d(47.5,drive.getPoseEstimate().getY()-1,Math.PI))
//                                                .build()
//        );
//        armDown();
//        delay(0.5);
//        armUp();
//        //go back 1
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .splineTo(new Pose2d(-36-24, drive.getPoseEstimate().getY()-2,Math.PI))
//                .build()
//        );
//        armDown();
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .strafeRight(4)
//                .build()
//        );
//        clawGrab();
//        delay(0.5); //pickup 2
//        armUp();
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .strafeLeft(4)
//                        .build()
//        );
//        //head to foundation 2
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                        .reverse()
//                        .splineTo(new Pose2d(50, drive.getPoseEstimate().getY()+3,Math.PI))
//                .build()
//        );
//        clawRelease();
//        delay(0.5);
//        armDown();
//        // go back 2
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .splineTo(new Pose2d(-28, drive.getPoseEstimate().getY()-4,Math.PI))
//                .build()
//        );
//        armDown();
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .strafeRight(6)
//                .build()
//        );
//        clawGrab();
//        delay(0.5);
//        armUp();
//        drive.followTrajectorySync(
//                drive.trajectoryBuilder()
//                .strafeLeft(6)
//                .build()
//        );
        telemetry.addData("kill the","motors");
        telemetry.update();
    }


    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Vuforia.setFrameFormat(PIXEL_FORMAT.GRAYSCALE, true);
        vuforia.setFrameQueueCapacity(1);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void launchStone(SampleMecanumDriveBase drive) {
        clawRelease();
        delay(0.2);
        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .strafeRight(6)
                .build()
        );
        armDown();
        delay(0.3);
        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .strafeLeft(6)
                .build()
        );
        armUp();

    }

    private void strafeAndGrab(SampleMecanumDriveBase drive) {
        armDown();
        clawRelease();
        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .strafeRight(6)
                .build()
        );
        delay(0.5);
        clawGrab();
        delay(0.5);
        armUp();
        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .strafeLeft(6)
                .build()
        );
        clawRelease();
        delay(0.3);
        clawGrab();
    }

    private boolean getSkystone() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) continue;
                    stoneLeft = (int) (recognition.getLeft());
                    stoneRight = (int) recognition.getRight();
                    stoneBottom = (int) recognition.getBottom();
                    stoneTop = (int) recognition.getTop();
                    stoneHeading = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                    return true;
                }
            }
        }
        return false;
    }

    //returns the column
    private int findSkyStone(byte[] pixelArray, int left, int right, int top, int bot) {
        int minC = 0;
        int row = (top + bot) / 2;
        int l = row * 640 + left;
        int r = row * 640 + right;
        byte min = pixelArray[l];
        for(int i = l; i < r; i++) {
            if(min < pixelArray[i]) {
                min = pixelArray[i];
                minC = i;
            }
        }
        return minC - (row * 640);
    }

    private int[] findSkyStone2(byte[] pixelArray, int left, int right, int top, int bot) {
        int row = (top + bot) / 2;
        int l = row * 640 + left;
        int r = row * 640 + right;
        byte[] vals = new byte[r - l + 1];
        int min = pixelArray[l];
        int avg = 0;
        int minC = l;
        for(int i = l; i < r; i++) {
            vals[i - l] = pixelArray[i];
            avg += pixelArray[i];
            if(min < pixelArray[i]) {
                minC = i;
                min = pixelArray[i];
            }
        }
        if(r - l != 0) avg /= (r - l);
        // dark values will be positive for the purposes of the algorithm
        int[] diff = new int[r - l + 1];
        for(int i = l; i <= r; i++) {
            diff[i - l] = -(pixelArray[i] - avg);
        }

        int max = diff[0];
        int[] sum = new int[diff.length];
        int beginMax = 0, endMax = 0, begin = 0, end = 0;
        for(int i = 0; i < diff.length; i++) {
            if(diff[i] < 0) {
                sum[i] = 0;
                begin = i + 1;
                continue;
            }
            if(i > 0)
                sum[i] = sum[i - 1] + diff[i];
            end = i;
            if(max < sum[i]) {
                max = sum[i];
                beginMax = begin;
                endMax = i;
            }
            else if(max == sum[i]) {
                if((endMax - beginMax) < (end - begin)) {
                    beginMax = begin;
                    endMax = end;
                }
                // else if((beginMax - endMax) == (begin - end))
            }
        }
        int[] arr = new int[4];
        arr[0] = beginMax + left;
        arr[1] = endMax + left;
        arr[2] = minC;
        arr[3] = (arr[0] + arr[1]) / 2;
        return arr;
    }

    private void displayStoneInfo(Telemetry telemetry) {
        telemetry.addData("stoneLeft", stoneLeft);
        telemetry.addData("stoneRight", stoneRight);
        telemetry.addData("stoneTop", stoneTop);
        telemetry.addData("stoneBottom", stoneBottom);
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