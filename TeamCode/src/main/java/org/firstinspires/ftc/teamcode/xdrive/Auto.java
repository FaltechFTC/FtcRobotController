package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Auto Generic", group = "7079")

public class Auto extends LinearOpMode {
    AutoBrain brain = new AutoBrain();

    @Override
    public void runOpMode() {
        brain.init(this);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        try {
            doMission();
        } catch (Exception e) {
            e.printStackTrace();
            telemetry.addLine("Exception in doMission()");

        }

        brain.robot.setDriveStop();
    }

    public void doMission() throws Exception {
        brain.autoNothing();
    }

}
