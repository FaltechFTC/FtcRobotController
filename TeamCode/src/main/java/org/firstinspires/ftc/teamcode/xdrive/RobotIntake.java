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
    static boolean useGantry = true;
    static boolean useCarousel = true;
    static boolean useDistanceSensor = false;
    static boolean debugTelemetry = false;
    private Telemetry telemetry = null;
    public Servo xyServo = null;
    public DcMotorSimple zMotor = null;
    private DcMotor zEncoder = null;
    //    public DcMotorSimple[] armArray = new DcMotorSimple[1];
    DcMotorSimple carousel = null;
    public int[] curPos = new int[4];
    public BNO055IMU imu = null;
    public DistanceSensor distanceSensor;
    public double magnetTime = 500;
    static public float carouselDirection = 1;
    boolean maintArm = false;
    boolean maintTimeout = false;
    public ElapsedTime carouselTimer = null;
    public ElapsedTime magnetTimer = null;
    boolean isClawOpen = false;
    double clawPosition = 0;
    double magnetPosition = 0;
    double carouselPower = 0;
/* we might need to leave this code for the arm here so that we can use it later is we are using
a claw system*/

    //    public DcMotor  leftArm     = null;
    Servo magnet = null;
    Servo claw = null;
    public double zPosition = 0;
    public double xyPosition;

    public static double MAGNET_ENGAGE_POS = 0.66;
    public static double MAGNET_RELEASE_POS = 0.35;
    public static double CLAW_OPEN_POS = 0.7;
    public static double CLAW_CLOSE_POS = 0.25;
    public static double maxUpPower = 0.7;
    public static double maxDownPower = -0.2;
    public static double verticalPowerConstant = .009;
    public static double MAX_HPOS = .72, MIN_HPOS = 0;
    public static double MAX_VPOS = 3400, MIN_VPOS = 0;
    public static double ARM_TOLERANCE = 3;


    public static double ARM_INTAKE_POS = 65.0;
    public static double ARM_LAYER1_POSZ = 678.0;
    public static double ARM_LAYER2_POSZ = 1453.0;
    public static double ARM_LAYER3_POSZ = 2547.0;
    public static double ARM_LAYER1_POSXY = 0.2;
    public static double ARM_LAYER2_POSXY = 0.5;
    public static double ARM_LAYER3_POSXY = 0.8;
    public static double ARM_PARK_POS = 800.0;

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
        magnet = hwMap.get(Servo.class, "magnet");
        claw = hwMap.get(Servo.class, "claw");
        xyServo = hwMap.get(Servo.class, "xyServo");
        zEncoder = hwMap.get(DcMotor.class, "frdrive");
        zEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        xyPosition = xyServo.getPosition();


        magnetEngage();
    }


    public int getArmPosition() {
        return zEncoder.getCurrentPosition();
    }

    public void setGantryPosition(double zPos, double xyPos, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        boolean done = false;
        setZPosition(zPos);
        setXYPosition(xyPos);
        while (timer.seconds() < timeoutSeconds && !done) {
            done = updateGantry();
        }
    }

    //lets go to halal bros
    //
    public boolean updateGantry() {
        boolean done = false;

        int currentPositionZ = (int) -zEncoder.getCurrentPosition();
        int errorz = ((int) zPosition) - currentPositionZ;
        done = Math.abs(errorz) < ARM_TOLERANCE;
        double p = 0;
        if (done) {
            p = 0;
        } else {
            p = verticalPowerConstant * errorz;
            p = Utility.clipToRange(p, maxUpPower, maxDownPower);
        }
        zMotor.setPower(p);

        telemetry.addLine()
                .addData("GantryPositionZ:", currentPositionZ)
                .addData("errorZ:", errorz)
                .addData("p", p)
                .addData("tgtZ:", zPosition);
        return done;
    }

    public void carouselMaint() {
        if (useCarousel && carouselTimer != null) {
            double m = carouselTimer.milliseconds();
            if (m < 600) carouselPower = ((m / 600) * .75 + .25);
            else if (m < 1350) {
                carouselPower = .85;
            } else {
                carouselPower = 0;
                carouselTimer = null;
            }
            setCarouselPower(carouselPower * carouselDirection);
        }
    }

    public void carouselStart(boolean direction) {
        carouselTimer = new ElapsedTime();
        carouselDirection = direction ? -1 : 1;
    }

    public void magnetMaint() {
        if (magnetTimer != null) {
            if (magnetTimer.milliseconds() > magnetTime) {
                magnetEngage();
                magnetTimer = null;
            }
        }
        magnet.setPosition(magnetPosition);
    }

    public void magnetStart(double magnetms) {
        magnetTimer = new ElapsedTime();
        magnetTime = magnetms;
        magnetRelease();
    }

    /*
        update() is called by drive.update() to maintain motors and timer based controls
     */
    public void update() {
        updateGantry();
        carouselMaint();
        magnetMaint();
    }

    public void setGantryPosition(double zpos, double xypos) {
        setXYPosition(xypos);
        setZPosition(zpos);
    }

    public double setXYPosition(double pos) {
        xyPosition = Utility.clipToRange(pos, MAX_HPOS, MIN_HPOS);
        xyServo.setPosition(xyPosition);
        telemetry.addData("xyPos", xyPosition);
        return xyPosition;
    }

    public double getXYPosition() {
        return xyPosition;
    }

    public double setZPosition(double pos) {
        zPosition = Utility.clipToRange(pos, MAX_VPOS, MIN_VPOS);
//        updateGantry(); // TODO: perhaps shouldn't be called here and should be called from update() only?
        return zPosition;
    }

    public double getZPosition() {
        return zPosition;
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
    public void reportIntake() {
            telemetry.addLine()
                    .addData("Carousel Power", "%.3f", carouselPower)
                    .addData("Claw Pos", "%.3f", clawPosition)
                    .addData("Magnet Pos", "%.3f", magnetPosition);
    }
    public void reportDistance() {
        if (useDistanceSensor) {
            telemetry.addData("Distance Sensor Reading:", distanceSensor.getDistance(DistanceUnit.INCH));
        }
    }

    public void clawOpen() {
        claw.setPosition(CLAW_OPEN_POS);
        isClawOpen = true;
        clawPosition = CLAW_OPEN_POS;
    }

    public void magnetEngage() {
        magnet.setPosition(MAGNET_ENGAGE_POS);
        magnetPosition = MAGNET_ENGAGE_POS;
    }

    public void clawClose() {
        claw.setPosition(CLAW_CLOSE_POS);
        isClawOpen = false;
        clawPosition = CLAW_CLOSE_POS;
    }

    public void magnetRelease() {
        magnet.setPosition(MAGNET_RELEASE_POS);
        magnetPosition = MAGNET_RELEASE_POS;
    }

    public void clawToggle() {
        if (isClawOpen) {
            clawClose();
        }
        else clawOpen();
    }

    public void setCarouselPower(double power) {
        carouselPower = power;
        carousel.setPower(carouselPower);
    }
}