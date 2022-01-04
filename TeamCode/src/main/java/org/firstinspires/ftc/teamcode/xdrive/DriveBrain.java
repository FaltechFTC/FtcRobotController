package org.firstinspires.ftc.teamcode.xdrive;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Pid;
import org.firstinspires.ftc.teamcode.Pose;
import org.firstinspires.ftc.teamcode.Utility;

@Config
public class DriveBrain {

    Robot robot;
    VisionBrain vision;
    OpMode opmode;
    private final ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime carouselTimer = null;
    public ElapsedTime pusherTimer = null;
    static final double P_DRIVE_COEFF = 0.15;
    static final double P_TURN_COEFF = 0.1;
    static final double HEADING_THRESHOLD = 1;
    int targetArmPos = 0;
    boolean maintArm = false;
    boolean maintTimeout = false;
    float carouselDirection = 1;
    public double pushTime = 500;

    public DriveBrain(Robot therobot, OpMode theopmode) {
        robot = therobot;
        opmode = theopmode;
    }

    public void driveTime(double forward, double strafe, double rotate, double power, double timeoutSeconds) {
        robot.setDrive(forward, strafe, rotate, power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeoutSeconds)) {
            opmode.telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            opmode.telemetry.update();

        }
        robot.setDriveStop();
    }

    public boolean opModeIsActive() {
        if (opmode instanceof LinearOpMode)
            return ((LinearOpMode) opmode).opModeIsActive();
        else return false;
    }

    public void driveDistance(double inches, double power, double timeoutSeconds) {
        ElapsedTime runtimeBusy = new ElapsedTime();
        runtime.reset();
        int clicks = 0;
        clicks = robot.convertInchesToCounts(inches);
        double forward = Math.abs(power);

        if (inches < 0) forward = forward;
        if (opModeIsActive()) {
            robot.setDriveDeltaPos(clicks, forward);
        }
        while (opModeIsActive() && (runtime.seconds() < timeoutSeconds) && (robot.isDriveBusy() || runtimeBusy.seconds() < 0.25)) {
            if (robot.isDriveBusy()) {
                runtimeBusy.reset();
            }
            maint();
            //Display it for the driver.
            //opmode.telemetry.addData("Path1", "Running to %7d :%7d", robot.convertCountsToInches(inches));
            //   opmode.telemetry.addData("Path2", "Running at %7f :%7f", robot.getCurPos());
            // opmode.telemetry.update();
        }

        robot.setDriveStop();
        robot.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void gyroDrive(double speed, double distance, double angle) {
        int moveCounts;
        int[] newTarget = new int[3];
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (robot.convertInchesToCounts(distance));

            robot.setDriveDeltaPos(moveCounts, .5);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.setDrivePowersTank(speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.isDriveBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.setDrivePowersTank(leftSpeed, rightSpeed);

                // Display drive status for the driver.
                opmode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                opmode.telemetry.addData("Target", "%7d", newTarget);
                opmode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                opmode.telemetry.update();
            }

            // Stop all motion;
            robot.setDriveStop();
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.getHeading(AngleUnit.DEGREES);
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            opmode.telemetry.update();
        }

        // Stop all motion;
        robot.setDriveStop();
    }

    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opmode.telemetry.update();
        }
    }

    public boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setDrivePowersTank(leftSpeed, rightSpeed);

        // Display it for the driver.
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        opmode.telemetry.update();

        return onTarget;
    }

    public boolean checkWhite(NormalizedRGBA color) {
        if (color.blue > 200 && color.red > 200 && color.green > 200)
            return true;
        return false;
    }

    public void driveToWhite(double power, double timeoutSeconds) {
        runtime.reset();

        double forward = power;

        if (opModeIsActive()) {
            robot.setDrive(forward, 0, 0, power);
        }

        NormalizedRGBA rgba = robot.getRGBA();
        boolean white = checkWhite(rgba);

        while (opModeIsActive() && (runtime.seconds() < timeoutSeconds) && !white) {
            rgba = robot.getRGBA();
            white = checkWhite(rgba);

            // Display it for the driver.
            opmode.telemetry.addData("white", "%b", white);
            opmode.telemetry.update();
        }
        carouselMaint();
        robot.setDriveStop();
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean rotateToHeadingAbsolute(double targetHeading, double tolerance, double power, double timeout) {
        boolean fail = true;
        ElapsedTime runtimeInTolerance = new ElapsedTime();
        runtime.reset();
        double currentHeading = robot.getHeading(AngleUnit.DEGREES);
        double headingError = Utility.wrapDegrees360(targetHeading - currentHeading);
        while ((runtime.seconds() < timeout) && ((Math.abs(headingError) > tolerance) || runtimeInTolerance.seconds() < 0.25)
                && opModeIsActive()) {
            if (Math.abs(headingError) > tolerance) {
                runtimeInTolerance.reset();
            }
            currentHeading = robot.getHeading(AngleUnit.DEGREES);
            headingError = Utility.wrapDegrees360(targetHeading - currentHeading);
            headingError = Utility.clipToRange(headingError, 45, -45);
            if (Math.abs(headingError) < Math.min(.5, tolerance)) {
                headingError = 0;
            }
            double rotationCorrection = (headingError / 45.00) * power;
            if (rotationCorrection > 0 && rotationCorrection < 0.1) {
                rotationCorrection = 0.1;
            }
            if (rotationCorrection < 0 && rotationCorrection > -0.1) {
                rotationCorrection = -0.1;
            }
            robot.setDrive(0, 0, rotationCorrection, 1);
            opmode.telemetry.addData("Target Heading", targetHeading);
            opmode.telemetry.addData("Current Heading", currentHeading);
            opmode.telemetry.addData("Rotation Correction:", rotationCorrection);
            opmode.telemetry.addData("Heading Error:", headingError);
            opmode.telemetry.addData("Seconds Passed:", runtime.seconds());
            opmode.telemetry.update();
            maint();
        }
        robot.setDriveStop();
        return runtime.seconds() < timeout;
    }


    public boolean rotateToHeadingRelative(double targetHeading, double tolerance, double power, double timeout) {
        targetHeading += robot.getHeading(AngleUnit.DEGREES);
        return rotateToHeadingAbsolute(targetHeading, tolerance, power, timeout);
    }

    public void carouselMoves(int direction) {
        if (robot.useCarousel) {
            robot.carousel.setPower(.3 * direction);
            sleep(2000);
            robot.carousel.setPower(.4 * direction);
            sleep(500);
            robot.carousel.setPower(1 * direction);
            sleep(800);
            robot.carousel.setPower(-1 * direction);
            sleep(350);
            robot.carousel.setPower(0);
        }
    }

    //    public void carouselStart() {
//        carouselTimer = new ElapsedTime();
//    }
    public void carouselMove() {
        carouselStart(true);
        if (carouselTimer != null) {
            if (carouselTimer.milliseconds() > 350) robot.carousel.setPower(.3);
            else if (carouselTimer.milliseconds() > 700) robot.carousel.setPower(.4);
            else if (carouselTimer.milliseconds() > 1500) robot.carousel.setPower(1);
            else {
                robot.carousel.setPower(0);
                carouselTimer = null;
            }
        }
    }
//    public void pusherStart() {
//        pusherTimer = new ElapsedTime();
//    }
//    public void pusherMove() {
//        pusherStart();
//        if (pusherTimer!=null) {
//            if (pusherTimer.milliseconds()>500) {
//                robot.pusherOpen();
//            }
//        }
//    }

    public void carouselMaint() {
        if (Robot.useCarousel && carouselTimer != null) {
            double m = carouselTimer.milliseconds();
            if (m < 600) robot.carousel.setPower((m / 600) * .75 + .25);
            else if (m < 1350) {
                robot.carousel.setPower(.85);
            } else {
                robot.carousel.setPower(0);
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
                robot.pusherOpen();
                pusherTimer = null;
            }
        }
    }

    public void pusherStart(double pushms) {
        pusherTimer = new ElapsedTime();
        pushTime = pushms;
        robot.pusherClose();
    }

    public double getBarcode() {
        return 0.5;
    }

    public void maint() {
        if (maintArm) {
            robot.setArmMotorPosition(targetArmPos);
        }
        carouselMaint();
        pusherMaint();
    }

    public void setArmMotorPosition(int pos) {
        targetArmPos = pos;
        maintArm = true;
    }

    public void maintTime(double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < timeoutSeconds && opModeIsActive()) {
            maint();
            sleep(1);
        }
    }

    // The following Pid() and smartDrive() was imported from internet resource (TODO: Insert URL)
    // and used as a template for continued work.
    final Pid pidDrivePrototype = new Pid(.001, 0.0, 0.0, -100, 100, -1.0, 1.0);
    final Pid pidRotatePrototype = new Pid(.001, 0.0, 0.0, -100, 100, -1.0, 1.0);

    public boolean smartDrive(Pose targetPose, double maxPower, double toleranceInches, double toleranceDegees, double timeoutSeconds) {
        ElapsedTime elapsed = new ElapsedTime();
        ElapsedTime elapsedInTolerance = new ElapsedTime();
        final double minToleranceSeconds = .1;
        int toleranceClicks = robot.convertInchesToCounts(toleranceInches);

        final double maxRotationPower = .5;

        Pid pidX = pidDrivePrototype.clone();
        pidX.setOutputLimits(-maxPower, maxPower);
        Pid pidY = pidDrivePrototype.clone();
        pidY.setOutputLimits(-maxPower, maxPower);
        Pid pidHeading = pidRotatePrototype.clone();
        pidY.setOutputLimits(-maxRotationPower, maxRotationPower);

        double lastTime = 0.0;

        targetPose.x = robot.convertInchesToCounts(targetPose.x);
        targetPose.y = robot.convertInchesToCounts(targetPose.y);

        boolean inTolerance = false;
        do {
            Pose currentPose = new Pose(robot.driveMotors2WheelX[0].getCurrentPosition(), robot.driveMotors2WheelY[0].getCurrentPosition(), robot.getHeading(AngleUnit.DEGREES));

            inTolerance = currentPose.isInTolerance(targetPose, toleranceClicks, toleranceDegees);
            if (!inTolerance) elapsedInTolerance.reset();

            // keep track of how much time per each loop
            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            double powerX = pidX.update(targetPose.x, currentPose.x, deltaTime);
            double powerY = pidY.update(targetPose.y, currentPose.y, deltaTime);
            double powerRot = pidHeading.update(targetPose.heading, currentPose.heading, deltaTime);

            robot.setDrive(powerY, powerX, powerRot, 1);

        } while (inTolerance && elapsedInTolerance.seconds() < minToleranceSeconds && elapsed.seconds() < timeoutSeconds && opModeIsActive());

        robot.setDriveStop();
        return inTolerance;
    }

    public void convertBarcode() {
        if (vision.returnvalue == 1) {
            setArmMotorPosition(Robot.ARM_LAYER1_POS);
        } else if (vision.returnvalue == 2) {
            setArmMotorPosition(Robot.ARM_LAYER2_POS);
        } else if (vision.returnvalue == 3) {
            setArmMotorPosition(Robot.ARM_LAYER3_POS);
        }
    }
}
