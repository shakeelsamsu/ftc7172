package org.firstinspires.ftc.teamcode.previous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Config
public class Glide {
    
    // Hardware
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    private DcMotorEx lift1 = null;
    private DcMotorEx lift2 = null;
    private ArrayMotor lift;
    int currLiftLevel = 0;
//    public static int l0 = 0;
//    public static int l1 = 4800;
//    public static int l2 = 7300;
//    public static int l3 = 11000;
//    public static int l4 = 15500;
//    public static int l5 = 19700;
//    public static int l6 = 24000;
//    public static int l7 = 28500;
//    public static int l8 = 32000;
//    public static int l9 = 36500;
//    public static int l10 = 40500;
//    public static int l11 = 45500;
//    public static int l12 = 50500;
//    public static int l13 = 55500;
//    public static int l14 = 60500;
    public static double RAMPING_ERROR = 5000;


    public static int[] liftLevels = {0,4800,7300,13000,18000,23000,28000,33000,38000,36500,40500,45500,50500,55500,60500};
    private double[] liftTimes = {0, 0.5, 0.6, 0.7, 0.8, 0.9, 
                                    1.0, 1.1, 1.2, 1.3,1.4,1.5,1.6,1.7,1.8,1.9,2.0};
    final static int MAX_HEIGHT = 14;
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
    public final double BAR_FORWARD = 0.23;    // when bar is holding a stone
    public final double BAR_BACK = 0.75; // when bar points in robot's forward direction, when it is idle
    double barPos = BAR_FORWARD;
    public double grabTime = 0;

    private Servo arm = null;
    private Servo claw = null;
    public static double ARM_UP = 0.79;
    public static double ARM_DOWN = 0.13;
    public static double CLAW_GRAB = 0.174;
    public static double CLAW_RELEASE = 0.69;

    
    private DistanceSensor lalign = null;
    private DistanceSensor stonedist = null;
    final double STONE_DIST_BAR = 1.6;
    final double STONE_DIST_GRAB = 0.3;
    final double GRAB_WAIT = 0.75;
    boolean seen = false;
    
    public double GANTRY_RETRACT = .995;
    public double GANTRY_EXTEND = 0.16;
    private double gantryPosition = GANTRY_RETRACT;
    private double gantryLock = 0;
    private double GANTRY_EXTEND_DELAY = 1;
    
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
    private final double TRACK_TICKS = 2332;
    private final double TICKS_PER_INCH = 1142.1815; //prev 190.a0
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

    public final int LEVEL_DEAD_ZONE = 20;
    
    ElapsedTime clock;

    double gantryStopTime = 0;
    // double gantryInOut = 0;
    
    double grabPower = 0;
    final double GRAB_TIME = 0.95;
    double grabStopTime = 0;
    boolean grabbed = false;
    
    public static enum Auto {IDLE, LIFT, HOME, GRAB, SCORE};
    Auto autoState = Auto.IDLE;
    
    public double ALIGN_STRAFE_GAIN = 0.6;
    
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
        // convey = hardwareMap.get(Servo.class, "conveyor");
        bar = hardwareMap.get(Servo.class, "bar");
        foundation = hardwareMap.get(Servo.class, "foundation");
        cap = hardwareMap.get(Servo.class, "capstone");

        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        
        lalign = hardwareMap.get(DistanceSensor.class, "ldist");
        stonedist = hardwareMap.get(DistanceSensor.class, "stonedist");
        elevatorLimit = hardwareMap.get(AnalogInput.class, "elevatorlimit");
        
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
        
        lift = new ArrayMotor(lift1, lift2, lift1, elevatorLimit);
        //accommodate for recent change
        for(int i = 0; i < liftLevels.length; i++)
            liftLevels[i] = (int) Math.round(liftLevels[i]);
        // gantryTimer = new ElapsedTime();

        //no longer used
        // inTimer = new ElapsedTime();

        lEncStart = this.leftPos();
        rEncStart = this.rightPos();
        
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

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
            t.addData("lift.targetPower", lift.getTargetPower());
            t.addData("lift.targetPosition", lift.getTargetPosition());
            t.addData("lift.currentPosition", lift.getCurrentPosition());
            t.addData("REVboreEncoder", rf.getCurrentPosition());
            t.addData("lift.status", lift.getStatus());
            t.addData("lift.power", lift.getPower());
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
            t.addData("ralign", 0);//ralign.getDistance(DistanceUnit.INCH));
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
        this.addTelemetryData(t, DISPLAY_ODOMETRY );
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
        return lift2.getCurrentPosition();
    }
    
    private int rightPos() {
        return -1*lift1.getCurrentPosition();
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
        lift.setTargetPower(power);
    }
    
    public void setLiftPosition(int position) {
        lift.setTargetPosition(position);
    }
    
    //goes to specific lift level
    public void setLiftLevel(int level) {
        if (level < 1) lift.setTargetLimit();
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
        return getLiftPosition() >= liftLevels[currLiftLevel] - LEVEL_DEAD_ZONE;
        //  && getLiftPosition() <= liftLevels[currLiftLevel] + LEVEL_DEAD_ZONE;
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
//        double now = clock.seconds();
//        double remaining = grabStopTime - now;
        barPos = BAR_FORWARD;
        if(inPower < 0)
            barPos = BAR_FORWARD;
        else if(stoneDist() < STONE_DIST_BAR) {
            barPos = BAR_BACK;
            if (elevatorLimit.getVoltage() < 1) setGantry(GANTRY_RETRACT);
            if (stoneDist() < STONE_DIST_GRAB &&!grabbed) {
                grabBlock();
                grabbed = true;
//                if(!seen) grabStopTime = now + GRAB_WAIT;
//                seen = true;
//                if(remaining > 0.5)
//                    letGo();
//                else if(remaining > 0) {
//                    grabBlock();
//                    seen = false;
//                    grabbed = true;
//                }
//                else {
//                    liftPower(0);
//                }
            }
            else if (stoneDist() > STONE_DIST_GRAB) {
                
            } //pull in gantry
//            if (!grabbed && getLiftLevel() == 0) {
//                liftPower(-.5);
//            }
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
    public boolean isGrabReady() {
        return(clock.seconds()+.37 > gantryLock);
    }
    
    public void setGantry(double pos) {
        gantryLock = clock.seconds() + .75;
        pos = Range.clip(pos,GANTRY_EXTEND,GANTRY_RETRACT);
        gantry.setPosition(pos);
    }

    public void moveGantry(double pos) {
        setGantry(gantry.getPosition()+pos);
    }
    
    public void setGrab(double pos) {
        pos = Range.clip(pos,.49,.916);
        grab.setPosition(pos);
    }
    
    public void moveGrab(double delta) {
        setGrab(grab.getPosition()+delta);
    }
    
    public void letGo() {
        setGrab(.7); // out
    }
    
    public void grabBlock() {
        setGrab(.915);
    }
    
    // CAPSTONE
    
    public void capstonePower(double pow) {
        cap.setPosition(pow);
    }

    // CLAW AND ARM

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
                if(isLiftAtTarget()) { 
                    setGantry(GANTRY_EXTEND);
                    setAuto(Auto.IDLE);
                }
                break;
            case HOME:
                if (isGrabReady()) grabBlock();
                if (isGantryIdle()) {
                    setLiftLevel(0);
                    setAuto(Auto.IDLE);
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
        }
    }
    
    public void cancel() {
        setAuto(Auto.IDLE);
    }
    
    public void update() {
         updatePosition();
        updateAuto();
        stoneUpdate();
        inUpdate();
        liftUpdate();
        // gantryUpdate();
    }
}