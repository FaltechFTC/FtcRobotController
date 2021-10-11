package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class faltechBotXDrive {
    static final boolean useColorSensor = false;
    static final boolean useIMU = true;

    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    public DcMotor[] driveMotors = new DcMotor[4];
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
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* setting up color sensor*/
    float colorSensorGain = 2;
    NormalizedColorSensor colorSensor = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

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
        // Define and Initialize Motors
        front_left  = hwMap.get(DcMotor.class, "fldrive");
        front_right = hwMap.get(DcMotor.class, "frdrive");
        back_left = hwMap.get(DcMotor.class,"bldrive");
        back_right = hwMap.get(DcMotor.class, "brdrive");
        driveMotors[0]=front_left;
        driveMotors[1]=front_right;
        driveMotors[2]=back_left;
        driveMotors[3]=back_right;
//        leftArm    = hwMap.get(DcMotor.class, "left_arm");
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
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);

    }
    public void setDrive(double forward, double strafe, double rotate, double power){
        //this function will combine wheel commands

        double[] wheelpowers = {
                (forward + strafe + rotate),
                (forward - strafe - rotate),
                (forward - strafe + rotate),
                (forward + strafe - rotate),
        };
        double max = Math.abs(wheelpowers[0]);
        for(int i = 0; i < wheelpowers.length; i++) {
            if ( max < Math.abs(wheelpowers[i]) ) max = Math.abs(wheelpowers[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < wheelpowers.length; i++) wheelpowers[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(wheelpowers[0]);
        front_right.setPower(wheelpowers[1]);
        back_left.setPower(wheelpowers[2]);
        back_right.setPower(wheelpowers[3]);
    }
    public void setDriveStop(){
        setDrive(0,0,0,0);
    }
    public void setRunMode(DcMotor.RunMode RunMode){
        front_left.setMode(RunMode);
        front_right.setMode(RunMode);
        back_left.setMode(RunMode);
        back_right.setMode(RunMode);
    }
    public void setDriveStopMode(boolean haltOnStop){
        if (haltOnStop){
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public double convertInchesToCounts(double inches){
        return COUNTS_PER_INCH * inches;
    }

    public double convertCountsToInches(double counts) {

        return counts / COUNTS_PER_INCH;
    }

    public boolean isDriveBusy() {
        boolean motor = false;
        if (front_left.isBusy()||front_right.isBusy()||back_right.isBusy()||back_left.isBusy()) {motor = true;}
        return motor;
    }
    public int[] getCurPos() {
        curPos[0] = front_left.getCurrentPosition();
        curPos[1] = front_right.getCurrentPosition();
        curPos[2] = back_left.getCurrentPosition();
        curPos[3] = back_right.getCurrentPosition();
        return curPos;
    }
    public void setDriveDeltaPos(int deltaPos, double power) {

        for (DcMotor m: driveMotors) {
            int curPos = m.getCurrentPosition();
            int newPos = curPos + deltaPos;
            m.setTargetPosition(newPos);

            // Turn On RUN_TO_POSITION
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotor m: driveMotors) {
            m.setPower(Math.abs(power));
        }
    }
    public void setDrivePowersTank(double leftPower, double rightPower) {
        front_left.setPower(leftPower);
        back_left.setPower(leftPower);
        front_right.setPower(rightPower);
        back_right.setPower(rightPower);
    }

    public NormalizedRGBA getRGBA() {
        if (useColorSensor) {
            return colorSensor.getNormalizedColors();
        }
        else {
            return new NormalizedRGBA();
        }
    }

    public double getHeading(AngleUnit angleUnit) {
        if (useIMU) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,angleUnit);
            return angles.firstAngle;
        }
        else{
            return 0;
        }

    }
    public void reportColor(){
      //  NormalizedRGBA colors = robotXDrive.getRGBA();
        //telemetry.addLine()
          //      .addData("Red", "%.3f", colors.red)
            ///    .addData("Green", "%.3f", colors.green)
               // .addData("Blue", "%.3f", colors.blue);
        //telemetry.update();
    }
}





