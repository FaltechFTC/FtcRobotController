package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Sped Auto", group = "7079")
public class TestAuto extends LinearOpMode {
    AutoBrainRR brain = new AutoBrainRR();

    //easter egg
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

    public void doMission(){
        brain.testAuto();
    }

}
