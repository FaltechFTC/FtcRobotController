package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Auto Blue 2", group = "7079")
public class AutoBlue2 extends LinearOpMode {
    AutoBrain brain = new AutoBrain();

    @Override
    public void runOpMode() {
        brain.init(this);


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

    public void doMission() throws Exception {

    }
}
