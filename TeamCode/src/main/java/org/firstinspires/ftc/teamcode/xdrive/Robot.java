package org.firstinspires.ftc.teamcode.xdrive;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {
    static final boolean useColorSensor = false;
    static final boolean useIMU = true;
    static final boolean useArm = true;
    static final boolean useCarousel = true;
    private Telemetry telemetry = null;
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    public DcMotor[] driveMotors = new DcMotor[4];
    public DcMotor[] driveMotors2WheelX = new DcMotor[2];
    public DcMotor[] driveMotors2WheelY = new DcMotor[2];
    public DcMotorSimple arm    = null;
    public DcMotorSimple carousel = null;
    public int[] curPos = new int[4];
    public BNO055IMU imu = null;
/* we might need to leave this code for the arm here so that we can use it later is we are using
a claw system*/

    //    public DcMotor  leftArm     = null;
//    public Servo    leftClaw    = null;
//    public Servo    rightClaw   = null;
//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_OUTPUT_REVOL = 537.7;
    static final double COUNTS_PER_INCH = (COUNTS_PER_OUTPUT_REVOL) / (WHEEL_DIAMETER_INCHES * Math.PI);

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* setting up color sensor*/
    float colorSensorGain = 2;
    NormalizedColorSensor colorSensor = null;

    public void init(HardwareMap ahwMap, Telemetry t) {
        hwMap = ahwMap;
        telemetry = t;
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
        if (useArm) {
            arm = hwMap.get(DcMotorSimple.class, "arm");
        }
        if (useCarousel) carousel = hwMap.get(DcMotorSimple.class, "carousel");

        // Define and Initialize Motors
        front_left = hwMap.get(DcMotor.class, "fldrive");
        front_right = hwMap.get(DcMotor.class, "frdrive");
        back_left = hwMap.get(DcMotor.class, "bldrive");
        back_right = hwMap.get(DcMotor.class, "brdrive");
        driveMotors[0] = front_right;
        driveMotors[1] = back_left;
        driveMotors[2] = front_left;
        driveMotors[3] = back_right;
        driveMotors2WheelY[0] = front_left;
        driveMotors2WheelY[1] = back_right;
        driveMotors2WheelX[0] = front_right;
        driveMotors2WheelX[1] = back_left;
        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        setDriveStop();
//        leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
//        leftClaw  = hwMap.get(Servo.class, "left_hand");
//        rightClaw = hwMap.get(Servo.class, "right_hand");
        //arm.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);

    }

    public void setRunMode(DcMotor.RunMode mode) {
        for (DcMotor m : driveMotors) {
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
        return (int) (COUNTS_PER_INCH * inches);
    }

    public double convertCountsToInches(int counts) {
        return counts / COUNTS_PER_INCH;
    }

    public boolean isDriveBusy() {
        boolean motor = false;
        if (front_left.isBusy() || front_right.isBusy() || back_right.isBusy() || back_left.isBusy()) {
            motor = true;
        }
        return motor;
    }

    public int[] getCurPos() {
        curPos[0] = front_left.getCurrentPosition();
        curPos[1] = front_right.getCurrentPosition();
        curPos[2] = back_left.getCurrentPosition();
        curPos[3] = back_right.getCurrentPosition();
        return curPos;
    }


    public void setDrivePower(double [] wheelpowers){
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
        setDrivePower(powers);
    }

    public void setDriveStop() {
        double[] powers = {0,0,0,0};
        setDrivePower(powers);
    }

    public void setDriveDeltaPos(int deltaPos, double power) {

        for (DcMotor m : driveMotors) {
            int curPos = m.getCurrentPosition();
            int newPos = curPos + deltaPos;
            m.setTargetPosition(newPos);

            // Turn On RUN_TO_POSITION
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotor m : driveMotors) {
            m.setPower(Math.abs(power));
        }
    }

    public void setDrivePowersTank(double leftPower, double rightPower) {
        double[] wheelpowers = { leftPower, rightPower, leftPower, rightPower };

        setDrivePower(wheelpowers); // apply the calculated values to the motors.
    }

    public void setTwoWheelDrive(double forward, double strafe, double rotate) {
        //this function will combine wheel commands
        double[] wheelpowers = {
                (strafe + rotate),
                (forward - rotate),
                (strafe + rotate),
                (forward - rotate),
        };

        setDrivePower(wheelpowers); // apply the calculated values to the motors.
    }

    public double[] calculateDrivePowersFSRSimple (double forward, double strafe, double rotate) {
       double[] powers = {
                (forward + strafe + rotate),
                (forward - strafe - rotate),
                (forward - strafe + rotate),
                (forward + strafe - rotate),
        };
        return powers;
    }

    public double[] calculateDrivePowers(double heading, double power, double rotate) {
        double m0, m1, m2, m3;
        m0 = power * -Math.sin(heading-(Math.PI/4))-rotate;
        m1 = power * Math.cos(heading+(Math.PI/4))-rotate;
        m2 = power * Math.sin(heading+(Math.PI/4))-rotate;
        m3 = power * -Math.cos(heading-(Math.PI/4))-rotate;
        double[] XDriveMotors = {m0, m1, m2, m3};
        return XDriveMotors;
    }

    public double[] calculateDrivePowersFSR (double gamepadx, double gamepady, double rotate) {
        double heading = Math.atan2(gamepadx,gamepady);
        double power = Math.sqrt((gamepadx*gamepadx)-(gamepady*gamepady));
        return calculateDrivePowers(heading,power,rotate);
    }

    public void reportDrivePowers(double [] powers) {
        telemetry.addLine()
                .addData("XDrive Power FL",powers[0])
                .addData("FR", powers[0])
                .addData("BL", powers[0])
                .addData("BR", powers[0]);
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

    public void reportColor(){
        if (useColorSensor) {
            NormalizedRGBA colors = getRGBA();
            telemetry.addLine()
                    .addData("Sensor Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
        }
    }

    public double getHeading(AngleUnit angleUnit) {
        if (useIMU) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit);
            return angles.firstAngle;
        } else {
            return 0;
        }

    }

}