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
import org.firstinspires.ftc.teamcode.Utility;

import java.util.List;

@Config
public class RobotIntake {
    static final boolean useColorSensor = false;
    static final boolean useIMU = true;
    public static double MAGNET_ENGAGE_POS = 0.25;
    public static double MAGNET_RELEASE_POS = 0.7;
    static boolean useArm = true;
    static boolean useCarousel = true;
    static boolean useDistanceSensor = false;
    static boolean debugTelemetry=false;
    private Telemetry telemetry = null;
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    public DcMotorSimple arm = null;
    //    public DcMotorSimple[] armArray = new DcMotorSimple[1];
    public DcMotorSimple carousel = null;
    public int[] curPos = new int[4];
    public BNO055IMU imu = null;
    public DistanceSensor distanceSensor;
/* we might need to leave this code for the arm here so that we can use it later is we are using
a claw system*/

    //    public DcMotor  leftArm     = null;
    public Servo intakePusher = null;
    public Servo intakeWrist = null;
    public double wristOffset = 0.40;
    public int armPosition = 0;
    public double maxUpPower = 0.6;
    public double maxDownPower = -0.3;



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
        if (useArm) arm = hwMap.get(DcMotorSimple.class, "arm");
        if (useCarousel) carousel = hwMap.get(DcMotorSimple.class, "carousel");
        if (useDistanceSensor) distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");


        // Define and initialize ALL installed servos.
        intakePusher = hwMap.get(Servo.class, "pusher");
        intakeWrist = hwMap.get(Servo.class, "wrist");

        magnetEngage();
        //wristMove();
    }











    public int getArmPosition() {
        return front_right.getCurrentPosition();
    }

    public void setArmPosition(int armPosition, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        boolean done=false;
       while (timer.seconds()<timeoutSeconds && !done)
       {
           done=setArmMotorPosition(armPosition);
       }
    }

    public boolean setArmMotorPosition(double pos) {
        armPosition = (int) pos;
        boolean done = false;
        int curentPosition = front_right.getCurrentPosition();
        int error = armPosition - curentPosition;
        done = Math.abs(error) < 3;
        double p = 0;
        if (done) {
            p = 0;
        } else {
            p = .012 * error;
            p = Utility.clipToRange(p, maxUpPower, maxDownPower);
        }
        arm.setPower(-p);
        telemetry.addLine()
                .addData("armPosition", curentPosition)
                .addData("error", error)
                .addData("p", p)
                .addData("tgt", armPosition);
        return done;
    }

    double maxWrist=.65, minWrist=0.0;
    public double setWristOffset(double offset) {
        wristOffset = Utility.clipToRange(offset, maxWrist, minWrist);
        return wristOffset;
    }
    public double calculateWristFromArm() {
        double pos=0.0;
        if (armPosition<100) pos=0;
        else if (armPosition<200) pos=.15;
        else if (armPosition>250) pos=armPosition/1000;
        pos = Utility.clipToRange(pos,maxWrist,minWrist);
        return pos;
    }

    public double getWristOffset() {
        return wristOffset;
    }

    public void wristMove() {
        intakeWrist.setPosition(wristOffset);
        telemetry.addData("wrist",wristOffset);
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