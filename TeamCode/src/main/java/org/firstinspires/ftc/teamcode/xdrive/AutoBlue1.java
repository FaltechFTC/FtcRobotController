package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Blue 1", group = "7079")
public class AutoBlue1 extends LinearOpMode {
    AutoBrain brain= new AutoBrain();

    @Override
    public void runOpMode() {
        brain.init(this);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        try {
            doMission();
        } catch (Exception e) {
            e.printStackTrace();
            telemetry.addLine("Exception in doMission()");
            // TODO log this properly
        }

        brain.robot.setDriveStop();
    }

    public void doMission() throws Exception
    {
        brain.autoPark1Blue(true,true);
    }

}