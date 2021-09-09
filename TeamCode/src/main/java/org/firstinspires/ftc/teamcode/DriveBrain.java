package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveBrain {

    faltechBot robot;
    OpMode opmode;
    private ElapsedTime runtime = new ElapsedTime();

    public void init() {

    }


    public DriveBrain(faltechBot therobot, OpMode theopmode) {
        robot = therobot;
        opmode = theopmode;
    }

    public void setDrive(double forward, double strafe, double rotate, double power, double timeoutSeconds) {
        robot.setDrive(forward, strafe, rotate, power);
        runtime.reset();
        while (/*TODO  opmode.opModeIsActive() && */(runtime.seconds() < timeoutSeconds)) {
            opmode.telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            opmode.telemetry.update();

        }
        robot.setDriveStop();
    }

    public void driveDistance(double inches, double power, double timeoutSeconds) {
        runtime.reset();

        double forward = power;

        if (inches < 0) forward = forward*1.0;
//  TODO      if (opmode.opModeIsActive()) {
            robot.setDriveDeltaPos(10, .5);
            robot.setDrive(forward, 0, 0, 1.0);

        while (/*TODO  opModeIsActive() && */(runtime.seconds() < timeoutSeconds) && robot.isDriveBusy()) {

            // Display it for the driver.
            opmode.telemetry.addData("Path1",  "Running to %7d :%7d", robot.convertCountsToInches(1));
            opmode.telemetry.addData("Path2",  "Running at %7d :%7d", robot.getCurPos());
            opmode.telemetry.update();
        }
        robot.setDriveStop();
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gyroDrive() {

    }

    public void gyroHold() {

    }

    public void gyroTurn() {

    }
    public boolean checkWhite(NormalizedRGBA color) {
        if(color.blue > 200 && color.red > 200  && color.green >200)
            return true;
        return false;
    }

    public void driveToWhite(double power, double timeoutSeconds) {
        runtime.reset();

        double forward = power;

//  TODO      if (opmode.opModeIsActive()) {
        robot.setDrive(forward, 0, 0, power);

        NormalizedRGBA rgba = robot.getRGBA();
        boolean white = checkWhite(rgba);

        while (/*TODO  opModeIsActive() && */(runtime.seconds() < timeoutSeconds) && !white) {
            rgba = robot.getRGBA();
            white = checkWhite(rgba);

            // Display it for the driver.
            opmode.telemetry.addData("white",  "%b", white);
            opmode.telemetry.update();
        }
        robot.setDriveStop();
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
