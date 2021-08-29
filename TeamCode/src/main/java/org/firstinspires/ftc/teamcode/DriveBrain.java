package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveBrain {

    faltechBot robot;
    OpMode opmode;
    private ElapsedTime runtime = new ElapsedTime();

    public void init() {

    }


    public void DriveBrain(faltechBot therobot, OpMode theopmode) {
        robot = therobot;
        opmode = theopmode;
    }

    public void driveFSRPTimed(double forward, double strafe, double rotate, double power, double timeoutSeconds) {
        robot.setDriveFSRP(forward, strafe, rotate, power);
        runtime.reset();
        while (/*TODO  opmode.opModeIsActive() && */(runtime.seconds() < timeoutSeconds)) {
            opmode.telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            opmode.telemetry.update();

        }
        robot.setDriveStop();
    }



}
