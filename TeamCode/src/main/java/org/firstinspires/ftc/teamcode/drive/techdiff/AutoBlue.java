package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.techdiff.GlideConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.nio.ByteBuffer;
import java.util.List;

@Config
@Disabled
@Autonomous
public class AutoBlue extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private static double R_ARM_STOW = GlideConstants.R_ARM_STOW;
    private static double R_ARM_GRAB = GlideConstants.R_ARM_GRAB;
    private static double R_ARM_OVER = GlideConstants.R_ARM_OVER;
    private static double R_ARM_DROP = GlideConstants.R_ARM_DROP;

    public static double R_CLAW_STOW = GlideConstants.R_CLAW_STOW;
    public static double R_CLAW_GRAB = GlideConstants.R_CLAW_GRAB;
    public static double R_CLAW_RELEASE = GlideConstants.R_CLAW_RELEASE;

    private static double R_ROTATE_SIDE = GlideConstants.R_ROTATE_SIDE;
    private static double R_ROTATE_DEPOSIT = GlideConstants.R_ROTATE_DEPOSIT;
    private static double R_ROTATE_BACK = GlideConstants.R_ROTATE_BACK;

    private static double FOUNDATION_GRAB = GlideConstants.FOUNDATION_GRAB;
    private static double FOUNDATION_RELEASE = GlideConstants.FOUNDATION_RELEASE;

    //
    public static double L_ARM_STOW = GlideConstants.L_ARM_STOW;
    public static double L_ARM_GRAB = GlideConstants.L_ARM_GRAB;
    public static double L_ARM_OVER = GlideConstants.L_ARM_OVER;
    public static double L_ARM_DROP = GlideConstants.L_ARM_DROP;

    // done
    public static double L_CLAW_STOW = GlideConstants.L_CLAW_STOW;
    public static double L_CLAW_GRAB = GlideConstants.L_CLAW_GRAB;
    public static double L_CLAW_RELEASE = GlideConstants.L_CLAW_RELEASE;

    public static double L_ROTATE_SIDE = GlideConstants.L_ROTATE_SIDE;
    public static double L_ROTATE_DEPOSIT = GlideConstants.L_ROTATE_DEPOSIT;
    public static double L_ROTATE_BACK = GlideConstants.L_ROTATE_BACK;

    ConstantInterpolator constInterp = new ConstantInterpolator(0);
    ConstantInterpolator constInterp180 = new ConstantInterpolator(Math.toRadians(180));
    LinearInterpolator linInterp = new LinearInterpolator(0,Math.toRadians(90));

    enum State {
        TO_FOUNDATION,
        TO_QUARRY,
        TO_FINISH,
        DEFAULT
    }
    enum Claw {
        LEFT,
        RIGHT
    }

    public static final double[] STONES_X = {-29.5, -37.5, -45.5, -41, -52, -60};
    public static final int[][] STONE_OPTIONS = {{5, 0, 1}, {5, 2, 0}, {4, 1, 0}, {3, 0, 1}};

    private Servo rarm;
    private Servo rrotate;
    private Servo rclaw;
    private Servo larm;
    private Servo lclaw;
    private Servo lrotate;
    private Servo foundation;

    private DcMotor lift1, lift2;

    private SampleMecanumDriveBase drive;
    private State state;
    private Claw clawSide;
    private Claw CLAW_SIDE = clawSide.RIGHT;
    private ElapsedTime liftClock = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AROJMNH/////AAABmfj2l+LOQ01wteJSaDwD5yVh+yKqERKv2v+nfmUTuA9A3zwyrXAkEUuDSjKPZ5MUfmz0hP9KASiO3tvdoq2eHNAZDhIx8DyW6RDu6FbrGCgO8orfeVL+Ya7z5tPYSH++sgyHKN/ED2DLd2pM7tdSOZZepRxNdegqJDE6C1t6dzXMSbOcfCSuWqTmRQYSoO3VxeszFfrX80jm31A3t5m2KOPq0xUuKKHStEz72JW+JEfaRMKGcrszGCilBowD1sGtwcCzshXCBXsTbK88En//xB8EHjIYpA6s/awDLj3/RDo6HsrO9T2iVtpIHl8Q4325gunwdek4+VvDc9ST5Jl/1UJrD0CXpTPW0kkDwSPSCGcO";

    private int stoneLeft = 0;
    private int stoneRight = 0;
    private int stoneBottom = 0;
    private int stoneTop = 0;
    private double stoneHeading = 0;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    boolean detected = false;

    enum Mode {TF, AVG, MANUAL}

    ;
    boolean lastDUp = false;
    boolean lastDDown = false;
    boolean lastDRight = false;
    boolean lastDLeft = false;
    boolean lastA = false;
    boolean lastB = false;
    boolean lastY = false;
    boolean lastX = false;

    Mode mode = Mode.AVG;
    boolean selectRow = false;
    int row = 300;
    int manualPosition = 1;

    public void runOpMode() {
        timer = new ElapsedTime();
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        rarm = hardwareMap.get(Servo.class, "rarm");
        rrotate = hardwareMap.get(Servo.class, "rrotate");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        larm = hardwareMap.get(Servo.class, "larm");
        lrotate = hardwareMap.get(Servo.class, "lrotate");
        lclaw = hardwareMap.get(Servo.class, "lclaw");
        foundation = hardwareMap.get(Servo.class, "foundation");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int stonePos = 5;

        initVuforia();
        telemetry.update();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.update();
        if (tfod != null) {
            tfod.activate();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.log().add("\nUI Controls:" +
                "\nA: AVG + select row" +
                "\nB: AVG + auto row" +
                "\nY: TF" +
                "\nX: MANUAL" +
                "\nDPAD_UP: row + 5" +
                "\nDPAD_DOWN: row - 5" +
                "\nDPAD_LEFT and DPAD_RIGHT: MANUAL positions");
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("mode", (mode == Mode.TF ? "TF"
                    : (mode == Mode.AVG ? "AVG" : "MANUAL")));
            telemetry.addData("selectRow?", selectRow);
            telemetry.addData("selected row", row);
            telemetry.addData("position", stonePos = getPosition());
            telemetry.update();
        }
        liftClock.reset();
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.setPoseEstimate(new Pose2d(-36, 63, 0));
        delay(0.5);
        LsetRotate(L_ROTATE_SIDE);
        LsetClaw(L_CLAW_STOW);
        LsetArm(L_ARM_STOW);
        RsetRotate(R_ROTATE_SIDE);
        RsetArm(R_ARM_STOW);
        RsetClaw(R_CLAW_STOW);
        setFoundation(FOUNDATION_RELEASE);

        // First Pick-Up
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(STONES_X[STONE_OPTIONS[stonePos][0]], 39))
                        .build()
                , State.DEFAULT
        );
        strafeAndGrabRight(drive, Math.abs(drive.getPoseEstimate().getY())-32);
        drive.update();

        // Go to Foundation
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(10, 39))
                        .lineTo(new Vector2d(50, 44), linInterp)
                        .build()
                , State.TO_FOUNDATION);
        drive.update();

        // Move Foundation and deposit
        drive.followTrajectorySync(
                drive.trajectoryBuilderSlow()
                        .back(drive.getPoseEstimate().getY()-29.5)
                        .build()
        );
        setFoundation(FOUNDATION_GRAB);
        delay(0.5);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(24,60,Math.toRadians(180)))
                        .build()
        );
        setFoundation(FOUNDATION_RELEASE);
        deposit();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(7)
                        .back(20).build()
        );

        // Switch claw/arm sides
        CLAW_SIDE = clawSide.LEFT;
        RsetRotate(R_ROTATE_SIDE);
        RsetArm(R_ARM_STOW);
        RsetClaw(R_CLAW_STOW);
        LsetArm(L_ARM_STOW);
        LsetClaw(L_CLAW_STOW);

        // Go back and Second Pick-Up
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(12, 39))
                        .splineTo(new Pose2d(STONES_X[STONE_OPTIONS[stonePos][1]],39,Math.toRadians(180)), constInterp180)
                        .build()
                , State.TO_QUARRY);
        strafeAndGrabLeft(drive, Math.abs(drive.getPoseEstimate().getY())-32);
        drive.update();




        //deposit second stone
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-10, 39, Math.toRadians(180)))
                .splineTo(new Pose2d(49, 39, Math.toRadians(180)))
                .build()
                , State.TO_FOUNDATION
        );
        deposit();
        delay(.25);

        // Go back and Third Pick-Up
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(12,39,Math.toRadians(180)))
                        .splineTo(new Pose2d(STONES_X[STONE_OPTIONS[stonePos][2]], 39, Math.toRadians(180)))
                        .build()
                , State.TO_QUARRY);
        strafeAndGrabLeft(drive,Math.abs(drive.getPoseEstimate().getY())-32);
        drive.update();

        // Go to foundation 3
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .reverse()
                .splineTo(new Pose2d(-10,39, Math.toRadians(180)))
                        .splineTo(new Pose2d(49, 39, Math.toRadians(180)))
                .build()
                , State.TO_FOUNDATION);
        drive.update();

        deposit();
        delay(.25);
        followTrajectoryArmSync(
                drive.trajectoryBuilderFinish()
                        .splineTo(new Pose2d(4, 39, Math.toRadians(180)))
                        .build()
                , State.TO_FINISH
        );

    }


    public void deposit() {
        if (CLAW_SIDE == clawSide.RIGHT) {
            RsetRotate(R_ROTATE_DEPOSIT);
            RsetArm(R_ARM_DROP);
            RsetClaw(R_CLAW_RELEASE);
        }
        else {
            LsetRotate(L_ROTATE_DEPOSIT);
            LsetArm(L_ARM_DROP);
            LsetClaw(L_CLAW_RELEASE);
        }
    }

    public void strafeAndGrabRight(SampleMecanumDriveBase drive, double offset) {
        RsetArm(R_ARM_OVER);
        RsetClaw(R_CLAW_RELEASE);
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .strafeRight(offset)
                        .build()
                , State.DEFAULT
        );
        RsetArm(R_ARM_GRAB);
        delay(0.3);
        RsetClaw(R_CLAW_GRAB);
        delay(0.3);
        RsetArm(R_ARM_DROP);
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .strafeLeft(offset)
                        .build()
                , State.DEFAULT
        );
    }

    public void strafeAndGrabLeft(SampleMecanumDriveBase drive, double offset) {
        LsetArm(L_ARM_OVER);
        LsetClaw(L_CLAW_RELEASE);
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .strafeLeft(offset)
                        .build()
                , State.DEFAULT
        );
        LsetArm(L_ARM_GRAB);
        delay(0.3);
        LsetClaw(L_CLAW_GRAB);
        delay(0.3);
        LsetArm(L_ARM_DROP);
        followTrajectoryArmSync(
                drive.trajectoryBuilder()
                        .strafeRight(offset)
                        .build()
                , State.DEFAULT
        );
    }

    public void setFoundation(double pos) {
        foundation.setPosition(pos);
    }


    public void RsetArm(double p) {
        rarm.setPosition(p);
    }
    public void RsetClaw(double p) {
        rclaw.setPosition(p);
    }
    public void RsetRotate(double p) {
        rrotate.setPosition(p);
    }

    public void LsetArm(double p) {
        larm.setPosition(p);
    }
    public void LsetClaw(double p) {
        lclaw.setPosition(p);
    }
    public void LsetRotate(double p) {
        lrotate.setPosition(p);
    }

    public void delay(double time) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < time) {
            drive.update();
            flipIntakeUpdate();
        }
    }
    public void liftPower(double pow) {
        lift1.setPower(pow);
        lift2.setPower(pow);
    }
    public void flipIntakeUpdate() {
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (liftClock.seconds() < 0.5 && lift1.getCurrentPosition() > -8000)  {
            liftPower(0.5);
        } else if (liftClock.seconds() <1.0) {
            liftPower(0);
        } else if (liftClock.seconds() < 5.0 && lift1.getCurrentPosition() < -200) {
            liftPower(-0.5);
        } else {
            liftPower(0);
        }
    }


    // TODO: make state an argument, add a default case
    public void followTrajectoryArmSync(Trajectory t, State s) {
        drive.followTrajectory(t);
        while(!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.update();
            flipIntakeUpdate();
            switch (s) {
                case TO_FOUNDATION:
                    if (drive.getPoseEstimate().getX() > -40) {
                        if (CLAW_SIDE == clawSide.RIGHT) RsetRotate(R_ROTATE_BACK);
                        else LsetRotate(L_ROTATE_BACK);
                    }
                    break;
                case TO_QUARRY:
                    if (drive.getPoseEstimate().getX() < -15) {
                        LsetClaw(L_CLAW_RELEASE);
                        LsetArm(L_ARM_OVER);
                        LsetRotate(L_ROTATE_SIDE);
                    }
                    break;
                case TO_FINISH:
                    LsetRotate(L_ROTATE_SIDE);
                    delay(0.3);
                    LsetClaw(L_CLAW_STOW);
                    LsetArm(L_ARM_STOW);
                    break;
                case DEFAULT:
                    break;
            }
        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraName = hardwareMap.get(WebcamName.class, "blueCam");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Vuforia.setFrameFormat(PIXEL_FORMAT.GRAYSCALE, true);
        vuforia.setFrameQueueCapacity(1);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private boolean detected() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
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

    private boolean getSkystone() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT))
                        continue;
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

    private int getSkyStonePosAvg(int row, boolean manual) {
        Frame frame = null;
        Image image = null;
        try {
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException e) {

        }
        if (frame != null) {
            for (int i = 0; i < frame.getNumImages(); i++) {
                image = frame.getImage(0);
                ByteBuffer pixels = image.getPixels();
                byte[] pixelArray = new byte[pixels.remaining()];
                pixels.get(pixelArray, 0, pixelArray.length);
                if (detected())
                    detected = true;
                if (detected)
                    return findSkyStoneAvg(pixelArray, manual ? row : (stoneTop + stoneBottom) / 2);
            }
        }
        return 0;
    }

    private int getSkyStonePosTF() {
        int pos = 0;
        getSkystone();
        if (stoneHeading < -10)
            pos = 3;
        else if (stoneHeading > 10)
            pos = 1;
        else
            pos = 2;
        displayStoneInfo(telemetry);
        return pos;
    }

    private int findSkyStoneAvg(byte[] pixelArray, int row) {
        int l = row * 640;
        int r = (row + 1) * 640;
        int avg1 = 0, avg2 = 0, avg3 = 0;
        int n1 = 0, n2 = 0, n3 = 0;
        for (int i = l; i < r; i++) {
            if (i < l + 213) {
                avg1 += pixelArray[i] & 0xFF;
                n1++;
            } else if (i < l + 427) {
                avg2 += pixelArray[i] & 0xFF;
                n2++;
            } else {
                avg3 += pixelArray[i] & 0xFF;
                n3++;
            }
        }
        avg1 /= n1;
        avg2 /= n2;
        avg3 /= n3;
        telemetry.addData("row", row);
        telemetry.addData("avg1", avg1);
        telemetry.addData("avg2", avg2);
        telemetry.addData("avg3", avg3);
        telemetry.addData("pixelArray", pixelArray.length);
        // telemetry.update();
        int min = Math.min(Math.min(avg1, avg2), avg3);
        if (min == avg1)
            return 3;
        if (min == avg2)
            return 2;
        return 1;
    }

    public int getPosition() {
        int position = 0;
        boolean a = gamepad1.a;
        if (a && !lastA) {
            mode = Mode.AVG;
            selectRow = true;
        }
        lastA = a;
        boolean b = gamepad1.b;
        if (b && !lastB) {
            mode = Mode.AVG;
            selectRow = false;
        }
        lastB = b;
        boolean y = gamepad1.y;
        if (y && !lastY) {
            mode = Mode.TF;
            selectRow = false;
        }
        lastY = y;
        boolean x = gamepad1.x;
        if (x && !lastX) {
            mode = Mode.MANUAL;
            selectRow = false;
        }
        lastX = x;
        if (selectRow) {
            boolean DUp = gamepad1.dpad_up;
            if (DUp && !lastDUp) {
                row += 5;
            }
            lastDUp = DUp;
            boolean DDown = gamepad1.dpad_down;
            if (DDown && !lastDDown) {
                row -= 5;
            }
            lastDDown = DDown;

        }
        switch (mode) {
            case AVG:
                position = getSkyStonePosAvg(row, selectRow);
                break;
            case TF:
                position = getSkyStonePosTF();
                break;
            case MANUAL:
                boolean right = gamepad1.dpad_right;
                if (right && !lastDRight)
                    manualPosition = Range.clip(manualPosition + 1, 1, 6);
                lastDRight = right;
                boolean left = gamepad1.dpad_left;
                if (left && !lastDLeft)
                    manualPosition = Range.clip(manualPosition - 1, 1, 6);
                lastDLeft = left;
                position = manualPosition;
                break;
        }
        return position;
    }

    private void displayStoneInfo(Telemetry telemetry) {
        telemetry.addData("stoneLeft", stoneLeft);
        telemetry.addData("stoneRight", stoneRight);
        telemetry.addData("stoneTop", stoneTop);
        telemetry.addData("stoneBottom", stoneBottom);
        telemetry.addData("stoneHeading", stoneHeading);
    }

}
