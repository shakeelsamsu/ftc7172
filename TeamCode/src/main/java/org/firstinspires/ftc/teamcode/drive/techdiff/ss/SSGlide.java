package org.firstinspires.ftc.teamcode.drive.techdiff.ss;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public class SSGlide {

    // Hardware
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    public DcMotorEx lift1 = null;
    public DcMotorEx lift2 = null;
    private SSElevator lift;
    // boolean manualLift = false;
    double liftPower = 0;
    int currLiftLevel = 0;
    public static double RAMPING_ERROR = 5000;

    //                                0 1    2    3     4     5     6     7     8     9     0     1     2     3     4
    public static int[] liftLevels = SSGlideConstants.LIFT_LEVELS;
    private double[] liftTimes = {0, 0.5, 0.6, 0.7, 0.8, 0.9,
            1.0, 1.1, 1.2, 1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0};
    final static int MAX_HEIGHT = 14;
    final static int LIFT_EXTEND_POS = SSGlideConstants.LIFT_EXTEND_POS;
    // private double liftStoneTimer = 0;
    // private double LIFT_STONE_DELAY = 0.75;

    private DcMotorEx lin = null;
    private DcMotorEx rin = null;
    double inPower = 0;
    boolean jammed = false;
    double rMinV = 10000000;
    double lMinV = 10000000;

    private Servo grab = null;
    private Servo gantry = null;
    private Servo bar = null;
    private Servo foundation = null;
    private Servo cap = null;
    boolean capping = false;
    public final double BAR_FORWARD = 0.23;    // when bar is holding a stone
    public final double BAR_BACK = 0.75; // when bar points in robot's forward direction, when it is idle
    double barPos = BAR_FORWARD;
    public double grabTime = 0;
    public static double GRAB_POS = SSGlideConstants.GRAB_POS;
    public static double LETGO_POS = SSGlideConstants.LETGO_POS;

    private Servo rarm = null;
    private Servo rclaw = null;
    private Servo rrotate = null;

    private Servo larm = null;
    private Servo lclaw = null;
    private Servo lrotate = null;

    public static double R_ARM_STOW = SSGlideConstants.R_ARM_STOW;
    public static double R_ARM_GRAB = SSGlideConstants.R_ARM_GRAB;
    public static double R_ARM_OVER = SSGlideConstants.R_ARM_OVER;
    public static double R_ARM_DROP = SSGlideConstants.R_ARM_DROP;

    public static double R_CLAW_STOW = SSGlideConstants.R_CLAW_STOW;
    public static double R_CLAW_GRAB = SSGlideConstants.R_CLAW_GRAB;
    public static double R_CLAW_RELEASE = SSGlideConstants.R_CLAW_RELEASE;

    public static double R_ROTATE_SIDE = SSGlideConstants.R_ROTATE_SIDE;
    public static double R_ROTATE_DEPOSIT = SSGlideConstants.R_ROTATE_DEPOSIT;
    public static double R_ROTATE_BACK = SSGlideConstants.R_ROTATE_BACK;

    private static double FOUNDATION_GRAB = SSGlideConstants.FOUNDATION_GRAB;
    private static double FOUNDATION_RELEASE = SSGlideConstants.FOUNDATION_RELEASE;

    //
    public static double L_ARM_STOW = SSGlideConstants.L_ARM_STOW;
    public static double L_ARM_GRAB = SSGlideConstants.L_ARM_GRAB;
    public static double L_ARM_OVER = SSGlideConstants.L_ARM_OVER;
    public static double L_ARM_DROP = SSGlideConstants.L_ARM_DROP;

    // done
    public static double L_CLAW_STOW = SSGlideConstants.L_CLAW_STOW;
    public static double L_CLAW_GRAB = SSGlideConstants.L_CLAW_GRAB;
    public static double L_CLAW_RELEASE = SSGlideConstants.L_CLAW_RELEASE;

    public static double L_ROTATE_SIDE = SSGlideConstants.L_ROTATE_SIDE;
    public static double L_ROTATE_DEPOSIT = SSGlideConstants.L_ROTATE_DEPOSIT;
    public static double L_ROTATE_BACK = SSGlideConstants.L_ROTATE_BACK;


    private DistanceSensor lalign = null;
    private DistanceSensor stonedist = null;
    private DistanceSensor autodist = null;
    final double STONE_DIST_BAR = SSGlideConstants.STONE_DIST_BAR;
    final double STONE_DIST_GRAB = SSGlideConstants.STONE_DIST_GRAB;
    final double GRAB_WAIT = SSGlideConstants.GRAB_WAIT;
    boolean seen = false;

    public double GANTRY_RETRACT = SSGlideConstants.GANTRY_RETRACT;
    public double GANTRY_CAP = SSGlideConstants.GANTRY_CAP;
    public double GANTRY_EXTEND = SSGlideConstants.GANTRY_EXTEND;
    private double gantryPosition = GANTRY_RETRACT;
    private double gantryLock = 0;
    private double GANTRY_EXTEND_DELAY = SSGlideConstants.GANTRY_EXTEND_DELAY;

    private AnalogInput elevatorLimit = null;

    // imu Variables
    private BNO055IMU imu = null;
    private double imuHeading = 0;
    private double imuZLast = 0;
    private int imuZTurns = 0;

    // Encoder Variables
    public int lEncStart = 0;
    public int rEncStart = 0;
    public int sEncStart = 0;
    private double encHeading = 0; // heading calculated by encoders
    private int lDistStart = 0;
    private int rDistStart = 0;
    private int sDistStart = 0;

    // Encoder Constants
    private final double TRACK_TICKS = SSGlideConstants.TRACK_TICKS;
    private final double TICKS_PER_INCH = SSGlideConstants.TICKS_PER_INCH; //prev 190.a0
    public final int DISPLAY_ODOMETRY = 0x0001;
    public final int DISPLAY_IMU = 0x0002;
    public final int DISPLAY_LIFT = 0x0004;
    public final int DISPLAY_CALCTRACK = 0x80000000;
    public final int DISPLAY_ALL = 0xffffffff;
    public final int DISPLAY_GANTRY = 0x0008;
    public final int DISPLAY_GRAB = 0x0010;
    public final int DISPLAY_ALIGN = 0x0020;
    public final int DISPLAY_GENERAL = 0x0040;
    public final int DISPLAY_INTAKE = 0x0080;

    public final int LEVEL_DEAD_ZONE = SSGlideConstants.LEVEL_DEAD_ZONE;

    ElapsedTime clock;

    double gantryStopTime = 0;
    // double gantryInOut = 0;

    double grabPower = 0;
    final double GRAB_TIME = SSGlideConstants.GRAB_TIME;
    double grabStopTime = 0;
    boolean grabbed = false;

    public static enum Auto {IDLE, LIFT, HOME, GRAB, SCORE};
    Auto autoState = Auto.IDLE;

    public double ALIGN_STRAFE_GAIN = SSGlideConstants.ALIGN_STRAFE_GAIN;

    public void init(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lin = hardwareMap.get(DcMotorEx.class, "lin");
        rin = hardwareMap.get(DcMotorEx.class, "rin");

        grab = hardwareMap.get(Servo.class, "grabber");
        gantry = hardwareMap.get(Servo.class, "gantry");
        bar = hardwareMap.get(Servo.class, "bar");
        foundation = hardwareMap.get(Servo.class, "foundation");
        cap = hardwareMap.get(Servo.class, "capstone");

        rarm = hardwareMap.get(Servo.class, "rarm");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        rrotate = hardwareMap.get(Servo.class, "rrotate");

        larm = hardwareMap.get(Servo.class, "larm");
        lclaw = hardwareMap.get(Servo.class, "lclaw");
        lrotate = hardwareMap.get(Servo.class, "lrotate");

        // lalign = hardwareMap.get(DistanceSensor.class, "ldist");
        stonedist = hardwareMap.get(DistanceSensor.class, "stonedist");
        elevatorLimit = hardwareMap.get(AnalogInput.class, "elevatorlimit");
        // autodist = hardwareMap.get(DistanceSensor.class, "autodist");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lin.setDirection(DcMotor.Direction.FORWARD);
        rin.setDirection(DcMotor.Direction.FORWARD);
        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.FORWARD);
        gantry.setDirection(Servo.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new SSElevator(lift1, lift2, lift1, elevatorLimit);
        //accommodate for recent change
        // gantryTimer = new ElapsedTime();

        //no longer used
        // inTimer = new ElapsedTime();

        lEncStart = this.leftPos();
        rEncStart = this.rightPos();

        // BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        // params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // imu = hardwareMap.get(BNO055IMU.class, "imu");
        // imu.initialize(params);

        rarm = hardwareMap.get(Servo.class, "rarm");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        rrotate = hardwareMap.get(Servo.class, "rrotate");

        clock = new ElapsedTime();
        clock.reset();
    }

    // Motor Methods
    public void rmoveXYW(double rx, double ry, double rw) {
        lf.setPower(ry+rx-rw);
        rf.setPower(ry-rx+rw);
        lb.setPower(ry-rx-rw);
        rb.setPower(ry+rx+rw);
    }

    public void rmoveXYH(double rx, double ry, double rh) {
        double herror = rh - getImuHeading();
        double kP = (rx==0 && ry==0) ? 0.015 : 0.01;
        rmoveXYW(rx, ry, herror * kP);
    }

    public void setMotorPower(double lpower, double rpower) {
        lf.setPower(lpower);
        rf.setPower(rpower);
        lb.setPower(lpower);
        rb.setPower(rpower);
    }
    public void motorPosTemp(Telemetry t) {
        t.addData("rf", rf.getCurrentPosition());
        t.addData("rb", rb.getCurrentPosition());
        t.addData("lb", lb.getCurrentPosition());
        t.addData("lf", lf.getCurrentPosition());
        t.addData("lift2", lift2.getCurrentPosition());
        t.addData("lift1", lift1.getCurrentPosition());
        t.addData("lin", lin.getCurrentPosition());
        t.addData("rin", rin.getCurrentPosition());
        t.update();

    }

    public void resetEncoders() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lEncStart = 0;
        rEncStart = 0;
        sEncStart = 0;
        encHeading = 0;
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void addTelemetryData(Telemetry t, int display) {
        if ((display & DISPLAY_ODOMETRY) != 0) {
            t.addData("left.pos", this.leftPos());
            t.addData("right.pos", this.rightPos());
            t.addData("encHeading", encHeading);
            t.addData("strafe.pos", this.strafePos());
            t.addData("getDist", getDist());
        }
        if ((display & DISPLAY_IMU) != 0) {
            t.addData("imuHeading", imuHeading);
        }
        if ((display & DISPLAY_LIFT) != 0) {
            t.addData("lift.power", lift.getPower());
            t.addData("lift.targetPower", lift.getTargetPower());
            t.addData("lift.targetPosition", lift.getTargetPosition());
            t.addData("lift.currentPosition", lift.getCurrentPosition());
            t.addData("REVboreEncoder", rf.getCurrentPosition());
            t.addData("lift.status", lift.getState());
            t.addData("atTarget", isLiftAtTarget());
            t.addData("currLiftLevel", currLiftLevel);
            t.addData("elevatorlimit", elevatorLimit.getVoltage());
        }
        if ((display & DISPLAY_CALCTRACK) != 0 && imuHeading != 0) {
            int currLeft = this.leftPos();
            int currRight = this.rightPos();
            int headingDelta = (currLeft - lEncStart) - (currRight - rEncStart);
            t.addData("calculated Track", headingDelta / Math.toRadians(imuHeading));

        }
        if((display & DISPLAY_GANTRY) != 0) {
            t.addData("gantryPosition", gantry.getPosition());
            // t.addData("gantryClock", clock.seconds() > gantryStopTime);
            // t.addData("gantryStopTime", gantryStopTime);
            //t.addData("gantryLimitVoltage", gantryLimit.getVoltage());
        }
        if((display & DISPLAY_GRAB) != 0) {
            t.addData("grabPower", grabPower);
            t.addData("grabClock", clock.seconds() > grabStopTime);
            // t.addData("gantryStopTime", gantryStopTime);
            t.addData("stoneDist", stoneDist());
        }
        if ((display & DISPLAY_ALIGN) != 0) {
            t.addData("lalign", lalign.getDistance(DistanceUnit.INCH));
            t.addData("autodist", autodist.getDistance(DistanceUnit.INCH));//ralign.getDistance(DistanceUnit.INCH));
        }
        if((display & DISPLAY_GENERAL) != 0) {
            t.addData("state", autoState);
        }
        if((display & DISPLAY_INTAKE) != 0) {
            t.addData("linV", lin.getVelocity());
            t.addData("rinV", rin.getVelocity());
        }
    }

    public void displayStatus(Telemetry t) {
        this.addTelemetryData(t, DISPLAY_GENERAL + DISPLAY_LIFT + DISPLAY_GRAB + DISPLAY_ODOMETRY);
        t.update();
    }

    public void sleep(LinearOpMode o, double timeout) {
        timeout += clock.seconds();
        while (o.opModeIsActive()) {
            if (clock.seconds() > timeout) break;
            update();
            displayStatus(o.telemetry);
        }
    }


    // Encoder Methods
    private int leftPos() {
        return rin.getCurrentPosition();
    }

    private int rightPos() {
        return lift2.getCurrentPosition();
    }

    private int strafePos() {
        return lin.getCurrentPosition();
    }

    public void updatePosition() {
        // update imu data
        getImuHeading();

        int currLeft = this.leftPos();
        int currRight = this.rightPos();
        int headingDelta = (currRight - rEncStart) - (currLeft - lEncStart);


        encHeading = Math.toDegrees(headingDelta/TRACK_TICKS);
    }

    public double getImuHeading() {
        Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES);
        double z = angles.firstAngle;
        if (z < -140 && imuZLast > 140) imuZTurns++;
        if (z > 140 && imuZLast < -140) imuZTurns--;
        imuZLast = z;
        imuHeading = imuZTurns * 360 + z;
        return imuHeading;
    }

    public void resetStartDist() {
        lDistStart = this.leftPos();
        rDistStart = this.rightPos();
        sDistStart = this.strafePos();
    }

    public int getDist() {
        int currLeft = this.leftPos();
        int currRight = this.rightPos();
        int currStrafe = this.strafePos();

        return ((currLeft - lDistStart) + (currRight - rDistStart))/2 + (currStrafe - sDistStart);
    }

    void driveXYHTicks(LinearOpMode o, double rx, double ry, double rh, int dist, double stoptime) {
        stoptime += clock.seconds();
        resetStartDist();
        double rxp = rx;
        double ryp = ry;

        while (o.opModeIsActive()) {
            if (Math.abs(this.getDist()) > dist) break;
            if (clock.seconds() >= stoptime) break;
            int error = Math.abs(this.getDist() - dist);
            if(error < RAMPING_ERROR) {
                double ramp = .7/RAMPING_ERROR*error + 0.3;
                rxp = rx * ramp;
                ryp = ry * ramp;
            }
            this.rmoveXYH(rxp, ryp, rh);
            this.update();
            // displayStatus(o.telemetry);
        }
        this.rmoveXYW(0,0,0);
    }

    void driveXYHInch(LinearOpMode o, double rx, double ry, double rh, double inch, double stoptime) {
        driveXYHTicks(o, rx, ry, rh, (int)(inch * TICKS_PER_INCH), stoptime);
    }

    void driveXYHInch(LinearOpMode o, double rx, double ry, double rh, double inch, double stoptime, Telemetry t) {
        driveXYHTicks(o, rx, ry, rh, (int)(inch * TICKS_PER_INCH), stoptime);
        displayStatus(t);
    }

    //3200 right = 15.748"

    void driveXYWTicks(LinearOpMode o, double rx, double ry, double rh, int dist, double stoptime) {
        stoptime += clock.seconds();
        resetStartDist();
        while (o.opModeIsActive()) {
            if (Math.abs(this.getDist()) > dist) break;
            if (clock.seconds() >= stoptime) break;
            this.rmoveXYH(rx, ry, rh);
            this.update();
            displayStatus(o.telemetry);
        }
        this.rmoveXYW(0,0,0);
    }

    void driveXYWInch(LinearOpMode o, double rx, double ry, double rw, double inch, double stoptime) {
        driveXYWTicks(o, rx, ry, rw, (int)(inch * TICKS_PER_INCH), stoptime);
    }

    public double getAlignStrafePower(double pow) {
        double ld = lalign.getDistance(DistanceUnit.INCH);
        double rd = 0;//ralign.getDistance(DistanceUnit.INCH);
        if (ld < 2 && rd < 2) return 0;
        if (pow == 0) { // starting a new align run
            if (ld < 15 && rd > 2) return -0.6;
            return 0.6;
        }
        if (ld < 4 && pow > 0) return -pow * ALIGN_STRAFE_GAIN;
        if (rd < 4 && pow < 0) return -pow * ALIGN_STRAFE_GAIN;
        return pow;
    }

    public void stop() {
        rmoveXYW(0, 0, 0);
    }
    //LIFT CODE

    public void liftPower(double power) {
        lift.setTargetPower(liftPower = power);
    }

    public void setLiftPosition(int position) {
        lift.setTargetPosition(position);
    }

    //goes to specific lift level
    public void setLiftLevel(int level) {
        if (level < 1) lift.setTargetHome();
        else setLiftPosition(liftLevels[level]);
        currLiftLevel = level;
    }

    public int getLiftLevel() {
        return currLiftLevel;
    }

    public int getLiftPosition() {
        return lift.getCurrentPosition();
    }

    public boolean isLiftAtTarget() {
        return getLiftPosition() >= liftLevels[currLiftLevel] - LEVEL_DEAD_ZONE
                && getLiftPosition() <= liftLevels[currLiftLevel] + LEVEL_DEAD_ZONE;
    }

    public void autoHome(LinearOpMode o, double timeout) {
        timeout += clock.seconds();
        while (o.opModeIsActive()) {
            if (clock.seconds() > timeout) break;
            if (elevatorLimit.getVoltage() < 1) break;
            lift.setTargetPower(-0.3);
            update();
        }
        lift.setTargetPower(0);
        currLiftLevel = 0;
        update();
    }
    //INTAKE CODE

    public void inPower(double power) {
        inPower = power;
    }

    // public void inPower(double lpower, double rpower) {
    //     lInPower = lpower;
    //     rInPower = rpower;
    // }

    public int getLInPosition() {
        return lin.getCurrentPosition();
    }

    public int getRInPosition() {
        return rin.getCurrentPosition();
    }

    public void inUpdate() {
        double power = inPower;
        double rinV = Math.abs(rin.getVelocity());
        double linV = Math.abs(lin.getVelocity());
        if(rinV != 0) rMinV = Math.min(rinV, rMinV);
        if(linV != 0) lMinV = Math.min(linV, lMinV);
        jammed = inPower > 0 && (rinV < 800 || linV < 800);
//        if(jammed) {
//            rin.setPower(inPower);
//            lin.setPower(inPower);
//        }
//        else {
        lin.setPower(-inPower);
        rin.setPower(inPower);
//        }
    }



    //STONE-MANIPULATION CODE

    public double stoneDist() {
        return stonedist.getDistance(DistanceUnit.INCH);
    }

    public void setBar(double pos) {
        bar.setPosition(pos);
    }

    // public boolean shouldLiftRelease() {
    //     return (clock.seconds() > liftStoneTimer);
    // }

    public void stoneUpdate() {
        double now = clock.seconds();
        double remaining = grabStopTime - now;
        barPos = BAR_FORWARD;
        if(inPower < 0)
            barPos = BAR_FORWARD;
        else if(stoneDist() < STONE_DIST_BAR) {
            barPos = BAR_BACK;
            if (elevatorLimit.getVoltage() < 1) setGantry(GANTRY_RETRACT);
            if (stoneDist() < STONE_DIST_GRAB &&!grabbed) {
                grabBlock();
                grabbed = true;
                if(!seen) grabStopTime = now + GRAB_WAIT;
                seen = true;
                if(remaining > 0.3)
                    letGo();
                else if(remaining > 0) {
                    grabBlock();
                    seen = false;
                    grabbed = true;
                }
                else {
                    liftPower(0);
                }
            }
            else if (stoneDist() > STONE_DIST_GRAB) {

            } //pull in gantry
            if (!grabbed && getLiftLevel() == 0) {
                liftPower(-.5);
            }
        }
        else grabbed = false;
        setBar(barPos);
    }

    public void setFoundation(double position) {
        foundation.setPosition(position);
    }

    public void liftUpdate() {
        lift.update();
    }

    //GANTRY & GRABBER------------------------------------------------------------------------------------------------

    public boolean isGantryIdle() {
        return (clock.seconds() > gantryLock);
    }
    // public boolean isGrabReady() {
    //     return(clock.seconds()+.37 > gantryLock);
    // }

    public void setGantry(double pos) {
        if(Math.abs(gantry.getPosition() - pos) > 0.01) {
            gantryLock = clock.seconds() + SSGlideConstants.GANTRY_EXTEND_DELAY;
        }
        pos = Range.clip(pos,GANTRY_EXTEND,GANTRY_RETRACT);
        gantry.setPosition(pos);
    }

    public void moveGantry(double pos) {
        setGantry(gantry.getPosition()+pos);
    }

    public void setGrab(double pos) {
        pos = Range.clip(pos,GRAB_POS,LETGO_POS);
        grab.setPosition(pos);
    }

    public void moveGrab(double delta) {
        setGrab(grab.getPosition()+delta);
    }

    public void letGo() {
        setGrab(LETGO_POS);
    }

    public void grabBlock() {
        setGrab(GRAB_POS);
    }

    // CAPSTONE

    public void capstonePower(double pow) {
        cap.setPosition(pow);
    }

    // CLAW AND ARM

    public void setRArm(double p) {
        rarm.setPosition(p);
    }

    public void setRClaw(double p) {
        rclaw.setPosition(p);
    }

    public void setRRotate(double p) {
        rrotate.setPosition(p);
    }

    public void setLArm(double p) {
        larm.setPosition(p);
    }

    public void setLClaw(double p) {
        lclaw.setPosition(p);
    }

    public void setLRotate(double p) {
        lrotate.setPosition(p);
    }

    //AUTO CODE

    public void setAuto(Auto state) {
        autoState = state;
    }

    public Auto getAuto() {
        return autoState;
    }

    public void updateAuto() {
        switch(autoState) {
            case LIFT:
                if(isLiftAtTarget() && currLiftLevel < 10) {
                    setGantry(GANTRY_EXTEND);
                    setAuto(Auto.IDLE);
                }
                // if(currLiftLevel == 1) {
                //     if()
                // }
                break;
            case HOME:
                // if (isGrabReady()) grabBlock();
                if(lift.getCurrentPosition() < LIFT_EXTEND_POS) {
                    setLiftPosition(liftLevels[2]);
                }
                // else if(moved 6 in) {
                // setLiftPosition(liftLevels[2]);
                //
                // }
                else {
                    setGantry(GANTRY_RETRACT);

                    if (isGantryIdle() || liftPower != 0) {
                        setLiftLevel(0);
                        setAuto(Auto.IDLE);
                        setGrab(GRAB_POS);
                    }
                    else if(getDist() > 8000) {
                        setLiftPosition(liftLevels[2]);
                        setGrab(GRAB_POS);
                    }
                }
                // if (gantryPower < 1) {
                //     setLiftLevel(0);
                //     setAuto(Auto.IDLE);
                // }
                // else if(gantryPower < 1.5) {
                //     setLiftLevel(2);
                // }
                break;
            case GRAB:
                // if(grabPower < 1) {
                //     setAuto(Auto.IDLE);
                //     grabbed = true;
                // }
                break;
            case SCORE:
                // if(lift.getTargetPower() == 0) {
                //     //setGrab(-2);
                // }
                break;
            case IDLE:
                // if(manualLift && liftPower == 0) {
                // setLiftPosition(getLiftPosition());
                // }
                break;
        }
    }

    public void cancel() {
        setAuto(Auto.IDLE);
    }

    public void update() {
        // updatePosition();
        updateAuto();
        stoneUpdate();
        inUpdate();
        liftUpdate();
        // gantryUpdate();
    }
}