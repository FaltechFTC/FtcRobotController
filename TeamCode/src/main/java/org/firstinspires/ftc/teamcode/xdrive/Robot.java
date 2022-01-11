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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Utility;

import java.util.List;

@Config
public class Robot {
    static final boolean useColorSensor = false;
    static final boolean useIMU = true;
    public static double MAGNET_ENGAGE_POS = 0.25;
    public static double MAGNET_RELEASE_POS = 0.7;
    public static double zeroHeadingOffset = 0;
    static boolean useArm = true;
    static boolean useCarousel = true;
    static boolean useDistanceSensor = false;
    static boolean debugTelemetry = false;
    private Telemetry telemetry = null;
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    public DcMotor[] driveMotors = new DcMotor[4];
    public DcMotor[] driveMotors2WheelY = new DcMotor[2];
    public DcMotor[] driveMotors2WheelX = new DcMotor[2];
    public DcMotor[] driveMotorsMode = new DcMotor[3];
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


    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_OUTPUT_REVOL = 537.7;
    static final double COUNTS_PER_INCH = (22.0 / 16.0) * (COUNTS_PER_OUTPUT_REVOL) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final int ARM_INTAKE_POS = 65;
    static final int ARM_LAYER1_POS = 224;
    static final int ARM_LAYER2_POS = 353;
    static final int ARM_LAYER3_POS = 513;
    static final int ARM_PARK_POS = 800; //to-do: set to park position

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private DcMotor armMotorEncoder;
    LynxModule.BulkCachingMode bulkReadMode = LynxModule.BulkCachingMode.AUTO;

    private ElapsedTime period = new ElapsedTime();

    /* setting up color sensor*/
    float colorSensorGain = 2;
    NormalizedColorSensor colorSensor = null;

    public void init(HardwareMap ahwMap, Telemetry t) {
        hwMap = ahwMap;
        telemetry = t;

        setBulkReadMode(bulkReadMode);
        if (useColorSensor) {
            colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color");
            colorSensor.setGain(colorSensorGain);
        }
        if (useIMU) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            //change to default set of parameters go here
            imu.initialize(params);
        }
        if (useArm) arm = hwMap.get(DcMotorSimple.class, "arm");
        if (useCarousel) carousel = hwMap.get(DcMotorSimple.class, "carousel");
        if (useDistanceSensor) distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        // Define and Initialize Motors
        front_left = hwMap.get(DcMotor.class, "fldrive");
        front_right = hwMap.get(DcMotor.class, "frdrive");
        back_left = hwMap.get(DcMotor.class, "bldrive");
        back_right = hwMap.get(DcMotor.class, "brdrive");
        driveMotors[0] = front_right;
        driveMotors[1] = back_left;
        driveMotors[2] = front_left;
        driveMotors[3] = back_right;
        driveMotorsMode[0] = front_right;
        driveMotorsMode[1] = back_left;
        driveMotorsMode[2] = front_left;
        driveMotors2WheelY[0] = front_left;
        driveMotors2WheelY[1] = back_right;
        driveMotors2WheelX[0] = front_right;
        driveMotors2WheelX[1] = back_left;
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setDriveStop();

        // Set all motors to run without encoders.
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        intakePusher = hwMap.get(Servo.class, "pusher");
        intakeWrist = hwMap.get(Servo.class, "wrist");

        armMotorEncoder = hwMap.get(DcMotor.class, "frdrive");

        magnetEngage();
        //wristMove();
    }

    public void setBulkReadMode(LynxModule.BulkCachingMode mode) {
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(mode);
        }
    }

    public void setRunMode(DcMotor.RunMode mode) {
        for (DcMotor m : driveMotors2WheelY) {
            m.setMode(mode);
        }
    }

    public void setDriveStopModeBreak() {
        DcMotor.ZeroPowerBehavior mode = DcMotor.ZeroPowerBehavior.BRAKE;
        for (DcMotor m : driveMotors) {
            m.setZeroPowerBehavior(mode);
        }
    }

    public void setDriveStopModeFloat() {
        DcMotor.ZeroPowerBehavior mode = DcMotor.ZeroPowerBehavior.FLOAT;
        for (DcMotor m : driveMotors) {
            m.setZeroPowerBehavior(mode);
        }
    }

    public int convertInchesToCounts(double inches) {
        return (int) ((COUNTS_PER_INCH) * inches);
    }

    public double convertCountsToInches(int counts) {
        return (double) (counts / (COUNTS_PER_INCH));
    }

    public boolean isDriveBusy() {
        boolean motor = false;
        if (front_left.isBusy() || front_right.isBusy() || back_right.isBusy() || back_left.isBusy()) {
            motor = true;
        }
        return motor;
    }

    public void setDrivePower(double[] wheelpowers) {
        double max = Math.abs(wheelpowers[0]);
        for (int i = 0; i < wheelpowers.length; i++) {
            if (max < Math.abs(wheelpowers[i])) max = Math.abs(wheelpowers[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < wheelpowers.length; i++) wheelpowers[i] /= max;
        }

        front_left.setPower(wheelpowers[0]);
        front_right.setPower(wheelpowers[1]);
        back_left.setPower(wheelpowers[2]);
        back_right.setPower(wheelpowers[3]);

        reportDrivePowers(wheelpowers);
    }

    public void setDrive(double forward, double strafe, double rotate, double power) {
        double[] powers = calculateDrivePowersFSRSimple(forward, strafe, rotate);
        if (debugTelemetry) {
            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
        }
        setDrivePower(powers);
    }

    public void setDriveStop() {
        double[] powers = {0, 0, 0, 0};
        setDrivePower(powers);
    }

    public void setDriveDeltaPos(int deltaPos, double power) {

        for (DcMotor m : driveMotors2WheelY) {
            int curPos = m.getCurrentPosition();
            int newPos = curPos + deltaPos;
            m.setTargetPosition(newPos);

            // Turn On RUN_TO_POSITIOn
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(Math.abs(power));
        }

    }

    public void setDrivePowersTank(double leftPower, double rightPower) {
        double[] wheelpowers = {leftPower, rightPower, leftPower, rightPower};

        setDrivePower(wheelpowers); // apply the calculated values to the motors.
    }

    public void setTwoWheelDrive(double forward, double strafe, double rotate) {
        //this function will combine wheel commands
        double[] wheelpowers = {
                (strafe - rotate),
                (forward + rotate),
                (strafe - rotate),
                (forward + rotate),
        };

        setDrivePower(wheelpowers); // apply the calculated values to the motors.
    }

    public double[] calculateDrivePowersFSRSimple(double forward, double strafe, double rotate) {
        double[] powers = {
                (forward + strafe - rotate),
                (forward - strafe + rotate),
                (forward - strafe - rotate),
                (forward + strafe + rotate),
        };
        return powers;
    }

    public double[] calculateDrivePowers(double heading, double power, double rotate) {
        double m0, m1, m2, m3;
        m0 = power * -Math.sin(heading - (Math.PI / 4)) + rotate;
        m1 = power * Math.cos(heading + (Math.PI / 4)) + rotate;
        m2 = power * Math.sin(heading + (Math.PI / 4)) + rotate;
        m3 = power * -Math.cos(heading - (Math.PI / 4)) + rotate;
        double[] XDriveMotors = {m0, m1, m2, m3};
        return XDriveMotors;
    }

    public double[] calculateDrivePowersFSR(double gamepadx, double gamepady, double rotate) {
        double heading = Math.atan2(gamepadx, gamepady);
        double power = Math.sqrt((gamepadx * gamepadx) - (gamepady * gamepady));
        return calculateDrivePowers(heading, power, rotate);
    }

    public void reportDrivePowers(double[] powers) {
        telemetry.addLine()
                .addData("XDrive Power FL", powers[0])
                .addData("FR", powers[0])
                .addData("BL", powers[0])
                .addData("BR", powers[0]);
    }

    public int getArmPosition() {
        return front_right.getCurrentPosition();
    }

    public void setArmPosition(int armPosition, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        boolean done = false;
        while (timer.seconds() < timeoutSeconds && !done) {
            done = setArmMotorPosition(armPosition);
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

    double maxWrist = .65, minWrist = 0.0;

    public double setWristOffset(double offset) {
        wristOffset = Utility.clipToRange(offset, maxWrist, minWrist);
        return wristOffset;
    }

    public double calculateWristFromArm() {
        double pos = 0.0;
        if (armPosition < 100) pos = 0;
        else if (armPosition < 200) pos = .15;
        else if (armPosition > 250) pos = armPosition / 1000;
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

    public void reportEncoders() {
        int fl = front_left.getCurrentPosition();
        int fr = front_right.getCurrentPosition();
        int bl = back_left.getCurrentPosition();
        int br = back_right.getCurrentPosition();
        telemetry.addLine()
                .addData("Encoder FL", fl)
                .addData("FR", fr)
                .addData("BL", bl)
                .addData("BR", br);
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

    public double getHeading(AngleUnit angleUnit) {
        return getRawHeading(AngleUnit.DEGREES) - zeroHeadingOffset;
    }

    public double getRawHeading(AngleUnit angleUnit) {
        if (useIMU) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit);
            return angles.firstAngle;
        } else {
            return 0;
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

    public void setZeroHeading() {
        zeroHeadingOffset = getRawHeading(AngleUnit.DEGREES);
        telemetry.addData("Zero Heading Offset", zeroHeadingOffset);
    }

}