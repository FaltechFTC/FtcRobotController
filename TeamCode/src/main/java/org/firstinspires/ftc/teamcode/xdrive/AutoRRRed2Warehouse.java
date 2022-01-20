package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto RR Red 2", group = "7079")
public class AutoRRRed2Warehouse extends LinearOpMode {
    AutoBrainRR brain = new AutoBrainRR();

    @Override
    public void runOpMode() {
        brain.init(this);

//        telemetry.addData("Status", "Ready to run");    //
//        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        try {
            doMission();
        } catch (Exception e) {
            e.printStackTrace();
            telemetry.addLine("Exception in doMission()");
            // TODO log this properly
        }

        brain.drive.setMotorPowers(0, 0, 0, 0);
    }

    public void doMission() throws Exception {
        brain.red2warehouse();
    }
}
