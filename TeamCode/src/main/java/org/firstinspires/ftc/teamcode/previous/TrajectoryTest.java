package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private DistanceSensor autodist = null;
    public static double ARM_UP = 0.55;
    public static double ARM_OVER = 0.18;
    public static double ARM_IN = 0.79;
    public static double ARM_GRAB = .12;
    public static double ARM_DROP = 0.4;
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
        autodist = hardwareMap.get(DistanceSensor.class, "autodist");
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
        drive.setPoseEstimate(new Pose2d(-36,-63,Math.PI));
        ConstantInterpolator interp = new ConstantInterpolator(Math.PI);

        // pick up one
        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .lineTo(new Vector2d(-36,-42),interp)
                .build()
        );
        strafeAndGrab(drive,8);

        //go to foundation,  deposit, and comeback
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(47.5, -42, Math.PI))
                .build()
        );
        delay(0.5);
        telemetry.addData("autodist",drive.getAutoDistInstant());
        telemetry.update();
        dropStone(drive,7);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .splineTo(new Pose2d(-36-24,-42, Math.PI))
                .build()
        );

        //pick up 2
        strafeAndGrab(drive,drive.getLastAutoDist()-2);

        // go to foundation deposit and comeback 2
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                .splineTo(new Pose2d(47.5, -42, Math.PI))
                .build()
        );
        dropStone(drive,7);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .splineTo(new Pose2d(-36+10,-42, Math.PI ))
                .build()
        );
//
//        //pick up 3
        double sDist = drive.getLastAutoDist()-1;
        telemetry.addData("a",sDist);
        telemetry.update();
        if (sDist > 14) sDist = 5;
        strafeAndGrab(drive,sDist);

        // go to foundation 3
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                .splineTo(new Pose2d(47.5+10,-42, Math.PI))
                .build()
        );
        dropStone(drive, 7);
        while(!isStopRequested()) {
            telemetry.addData("dist", autodist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        delay(10);

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

    private void dropStone(SampleMecanumDriveBase drive, double offset) {
        armDrop();
        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .strafeRight(offset)
                .build()
        );
        clawRelease();
        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .strafeLeft(offset)
                .build()
        );
        armIn();
    }

    private void strafeAndGrab(SampleMecanumDriveBase drive, double offset) {
        armOver();
        clawRelease();
        delay(0.2);
        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .strafeRight(offset)
                .build()
        );
        armGrab();
        delay(0.3);
        clawGrab();
        delay(0.5);
        armUp();
        if (offset < 6) offset = 6;
        drive.followStrafeSync(
                drive.trajectoryBuilder()
                .strafeLeft(offset)
                .build()
        );
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

    public void armOver() {
        setArm(ARM_OVER);
    }

    public void armGrab() {
        setArm(ARM_GRAB);
    }

    public void clawGrab() {
        setClaw(CLAW_GRAB);
    }

    public void clawRelease() {
        setClaw(CLAW_RELEASE);
    }

    public void armIn() {
        setArm(ARM_IN);
    }

    public void armDrop() {
        setArm(ARM_DROP);
    }
}