package org.firstinspires.ftc.teamcode.drive.techdiff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.nio.ByteBuffer;
import java.util.List;

@Disabled
@Autonomous
public class SSVisionTest extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        telemetry.update();
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
        // waitForStart();
        while (!opModeIsActive()) {

            telemetry.addData("mode", (mode == Mode.TF ? "TF"
                    : (mode == Mode.AVG ? "AVG" : "MANUAL")));
            telemetry.addData("selectRow?", selectRow);
            telemetry.addData("selected row", row);
            telemetry.addData("position", getPosition());
            telemetry.update();
        }

        if (opModeIsActive()) {
            tfod.shutdown();

            if (tfod != null) {
                tfod.shutdown();
            }
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraName = hardwareMap.get(WebcamName.class, "redCam");
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
            pos = 1;
        else if (stoneHeading > 10)
            pos = 3;
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
            return 1;
        if (min == avg2)
            return 2;
        return 3;
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