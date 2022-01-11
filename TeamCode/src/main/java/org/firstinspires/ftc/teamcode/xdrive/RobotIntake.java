package org.firstinspires.ftc.teamcode.xdrive;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class RobotIntake {
    static final boolean useColorSensor = false;
    static final boolean useIMU = true;
    public static double MAGNET_ENGAGE_POS = 0.25;
    public static double MAGNET_RELEASE_POS = 0.7;
    static boolean useGantry = true;
    static boolean useCarousel = true;
    static boolean useDistanceSensor = false;
    static boolean debugTelemetry = false;
    private Telemetry telemetry = null;
    public Servo xyServo = null;
    public DcMotorSimple zMotor = null;
    private DcMotor zEncoder = null;
    //    public DcMotorSimple[] armArray = new DcMotorSimple[1];
    public DcMotorSimple carousel = null;
    public int[] curPos = new int[4];
    public BNO055IMU imu = null;
    public DistanceSensor distanceSensor;
    public double pushTime = 500;
    static public float carouselDirection = 1;
    boolean maintArm = false;
    boolean maintTimeout = false;
    public ElapsedTime carouselTimer = null;
    public ElapsedTime pusherTimer = null;
/* we might need to leave this code for the arm here so that we can use it later is we are using
a claw system*/

    //    public DcMotor  leftArm     = null;
    public Servo intakePusher = null;
    public Servo intakeWrist = null;
    public double wristOffset = 0.40;
    public double zPosition = 0;
    public double xyPosition = 0;
    public static double maxUpPower = 0.6;
    public static double maxDownPower = -0.3;
    public static double verticalPowerConstant = .012;


    static final int ARM_INTAKE_POS = 65;
    static final int ARM_LAYER1_POS = 224;
    static final int ARM_LAYER2_POS = 353;
    static final int ARM_LAYER3_POS = 513;
    static final int ARM_PARK_POS = 800; //to-do: set to park position

    /* local OpMode members. */
    HardwareMap hwMap = null;
    LynxModule.BulkCachingMode bulkReadMode = LynxModule.BulkCachingMode.AUTO;

    private ElapsedTime period = new ElapsedTime();

    /* setting up color sensor*/
    float colorSensorGain = 2;
    NormalizedColorSensor colorSensor = null;

    public void init(HardwareMap ahwMap, Telemetry t) {
        hwMap = ahwMap;
        telemetry = t;

        if (useIMU) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            //change to default set of parameters go here
            imu.initialize(params);
        }
        if (useGantry) zMotor = hwMap.get(DcMotorSimple.class, "zMotor");
        if (useCarousel) carousel = hwMap.get(DcMotorSimple.class, "carousel");
        if (useDistanceSensor) distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");


        // Define and initialize ALL installed servos.
        intakePusher = hwMap.get(Servo.class, "pusher");
        intakeWrist = hwMap.get(Servo.class, "wrist");
        xyServo = hwMap.get(Servo.class, "xyServo");
        zEncoder = hwMap.get(DcMotor.class, "frdrive");

        magnetEngage();
        //wristMove();
    }


    public int getArmPosition() {
        return zEncoder.getCurrentPosition();
    }

    public void setGantryPosition(int zPosition, int xyPosition, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        boolean done = false;
        setGantryPosition(zPosition, xyPosition);
        while (timer.seconds() < timeoutSeconds && !done) {
            done = updateGantry(zPosition);
        }
    }
    //lets go to halal bros
    //
    public boolean updateGantry(int zpos) {
        boolean done = false;
        int currentPositionZ = (int) zEncoder.getCurrentPosition();
        int errorz = (int) zpos - currentPositionZ;
        done = Math.abs(errorz) < 3;
        double p = 0;
        if (done) {
            p = 0;
        } else {
            p = verticalPowerConstant * errorz;
            p = Utility.clipToRange(p, maxUpPower, maxDownPower);
        }
        zMotor.setPower(-p);
        telemetry.addLine()
                .addData("GantryPositionZ:", currentPositionZ)
                .addData("errorZ:", errorz)
                .addData("p", p)
                .addData("tgtZ:", zPosition)
                .addData("tgtXY:", xyPosition);
        return done;
    }

    public void carouselMaint() {
        if (Robot.useCarousel && carouselTimer != null) {
            double m = carouselTimer.milliseconds();
            if (m < 600) carousel.setPower((m / 600) * .75 + .25);
            else if (m < 1350) {
                carousel.setPower(.85);
            } else {
                carousel.setPower(0);
                carouselTimer = null;
            }
        }
    }

    public void carouselStart(boolean direction) {
        //true =
        carouselTimer = new ElapsedTime();
        carouselDirection = direction ? -1 : 1;
    }

    public void pusherMaint() {
        if (pusherTimer != null) {
            if (pusherTimer.milliseconds() > pushTime) {
                pusherOpen();
                pusherTimer = null;
            }
        }
    }

    public void pusherStart(double pushms) {
        pusherTimer = new ElapsedTime();
        pushTime = pushms;
        pusherClose();
    }

    public void update () {

        if (maintArm) {
            setGantryPosition(0, 0);
        }
        carouselMaint();
        pusherMaint();
    }

    public void setGantryPosition(double zpos, double xypos) {
        zPosition = zpos;
        xyPosition = xypos;
        xyServo.setPosition(xyPosition);
        updateGantry((int) zPosition);
    }

    double maxWrist = .65, minWrist = 0.0;

    public double setWristOffset(double offset) {
        wristOffset = Utility.clipToRange(offset, maxWrist, minWrist);
        return wristOffset;
    }

    public double calculateWristFromArm() {
        double pos = 0.0;
        if (zPosition < 100) pos = 0;
        else if (zPosition < 200) pos = .15;
        else if (zPosition > 250) pos = zPosition / 1000;
        pos = Utility.clipToRange(pos, maxWrist, minWrist);
        return pos;
    }

    public double getWristOffset() {
        return wristOffset;
    }

    public void wristMove() {
        intakeWrist.setPosition(wristOffset);
        telemetry.addData("wrist", wristOffset);
    }

    public void wristMove(double pos) {
        intakeWrist.setPosition(pos);
    }


    public NormalizedRGBA getRGBA() {
        if (useColorSensor) {
            return colorSensor.getNormalizedColors();
        } else {
            return new NormalizedRGBA();
        }
    }

    public void reportColor() {
        if (useColorSensor) {
            NormalizedRGBA colors = getRGBA();
            telemetry.addLine()
                    .addData("Sensor Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue)
                    .addData("L", "%.3f", colors.alpha);
        }
    }

    public void reportDistance() {
        if (useDistanceSensor) {
            telemetry.addData("Distance Sensor Reading:", distanceSensor.getDistance(DistanceUnit.INCH));
        }
    }

    public void pusherOpen() {
        intakePusher.setPosition(MAGNET_ENGAGE_POS);
    }

    public void magnetEngage() {
        intakePusher.setPosition(MAGNET_ENGAGE_POS);
    }


    public void pusherClose() {
        intakePusher.setPosition(MAGNET_RELEASE_POS);
    }

    public void magnetRelease() {
        intakePusher.setPosition(MAGNET_RELEASE_POS);
    }

}