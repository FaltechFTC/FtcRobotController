package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class DriveBrainMecanum {

    faltechBotMecanum robot;
    OpMode opmode;

    private final ElapsedTime runtime = new ElapsedTime();

    static final double     P_DRIVE_COEFF           = 0.15;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     HEADING_THRESHOLD       = 1 ;

    public DriveBrainMecanum(faltechBotMecanum therobot, OpMode theopmode) {
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
        return true;
//        opmode.OpModeIsActive();
    }
    public void driveDistance(double inches, double power, double timeoutSeconds) {
        runtime.reset();

        double forward = power;

        if (inches < 0) forward = forward;
            if (opModeIsActive()) {
            robot.setDriveDeltaPos(10, .5);
            robot.setDrive(forward, 0, 0, 1.0);
        }
        while (opModeIsActive() && (runtime.seconds() < timeoutSeconds) && robot.isDriveBusy()) {

            // Display it for the driver.
            opmode.telemetry.addData("Path1", "Running to %7d :%7d", robot.convertCountsToInches(inches));
            opmode.telemetry.addData("Path2", "Running at %7d :%7d", robot.getCurPos());
            opmode.telemetry.update();
        }
        robot.setDriveStop();
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void gyroDrive(double speed, double distance, double angle) {
        int     moveCounts;
        int[] newTarget = new int[3];
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * robot.convertCountsToInches(distance));

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
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.setDrivePowersTank(leftSpeed, rightSpeed);

                // Display drive status for the driver.
                opmode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                opmode.telemetry.addData("Target",  "%7d",      newTarget);
                opmode.telemetry.addData("Actual",  "%7d",      robot.getCurPos());
                opmode.telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                opmode.telemetry.update();
            }

            // Stop all motion;
            robot.setDriveStop();
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroHold( double speed, double angle, double holdTime) {

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
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setDrivePowersTank(leftSpeed, rightSpeed);

        // Display it for the driver.
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public boolean checkWhite(NormalizedRGBA color) {
        if(color.blue > 200 && color.red > 200  && color.green >200)
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

        while (  opModeIsActive() && (runtime.seconds() < timeoutSeconds) && !white) {
            rgba = robot.getRGBA();
            white = checkWhite(rgba);

            // Display it for the driver.
            opmode.telemetry.addData("white",  "%b", white);
            opmode.telemetry.update();
        }

        robot.setDriveStop();
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

