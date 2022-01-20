package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Auto Blue 1", group = "7079")
public class AutoRRBlue1StorageUnit extends LinearOpMode {
    AutoBrainRR brain = new AutoBrainRR();

    @Override
    public void runOpMode() {
        brain.init(this);


        waitForStart();

        try {
            doMission();
        } catch (Exception e) {
            e.printStackTrace();
            telemetry.addLine("Exception in doMission()");

        }

        brain.drive.setMotorPowers(0,0,0,0);
    }

    public void doMission() throws Exception {
        brain.blue1unitpark();
    }

}
