package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", group = "7079")
//@Disabled
public class Auto extends LinearOpMode {
    Robot robot;
    DriveBrain driveBrain;
    VisionBrain vision;
    boolean useVision=false;

    String stage="Starting";

    private ElapsedTime runtime = new ElapsedTime();
    static final double POWER = 0.3;

    class BailException extends Exception {
         BailException(String msg){
            super(msg);
        }
    }

    public void runOpMode() {

        telemetry.addData("Status", "Initializing Robot");
        telemetry.update();

        robot = new Robot();
        robot.init(hardwareMap, telemetry);
        robot.setDriveStopModeBreak();

        telemetry.addData("Status", "Initializing Brain");
        telemetry.update();

        driveBrain = new DriveBrain(robot, this);

        if (useVision) {
            telemetry.addData("Status", "Initializing Vision");
            telemetry.update();

            vision = new VisionBrain();
            vision.init(this, false);
        }

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        try {
            runMission();
        } catch (BailException e) {
            telemetry.addData("Bailed!", e.getMessage());
            telemetry.update();
        } finally {
            robot.setDriveStop();
        }
    }

    private void runMission() throws BailException{

        boolean waitForInput=true;
        //demoDriveTime(waitForInput);
//    demoRotate(waitForInput);
        demoDriveDistance(waitForInput);
        telemetry.addData("Path", "Mission Complete");
        telemetry.update();

    }

    void demoDriveTime(boolean waitForInput) throws BailException {

        double speed = .3;
        double timeout = .5;

        stage="demoDriveTime";
        checkBail(waitForInput);
        driveBrain.driveTime(speed, 0, 0, timeout); // forward
        checkBail(waitForInput);
        driveBrain.driveTime(-speed, 0, 0, timeout); // back
        checkBail(waitForInput);
        driveBrain.driveTime(0, speed, 0, timeout); // right
        checkBail(waitForInput);
        driveBrain.driveTime(0, -speed, 0, timeout); // left

    }

    void demoRotate(boolean waitForInput) throws BailException {
        stage="demoRotate";

        double speed=.35;
        double timeout=3;
        double tolerance=2.0;
        double[] headings =  new double[] {0, 90, 180, 270, 45, 0};
        for (double heading : headings) {
            checkBail(waitForInput);
            driveBrain.rotateToHeadingAbsolute(heading, tolerance, speed, timeout);
        }
        timeout=4.0;
        tolerance=1.4;
        speed=.25;
        headings = new double[] {45, -45, 180, 0};
        for (double heading : headings) {
            checkBail(waitForInput);
            driveBrain.rotateToHeadingAbsolute(heading, tolerance, speed, timeout);
        }
    }
    void demoDriveDistance(boolean waitForInput) throws BailException {
        stage="demoDriveDistance";
        double speed=.3;
        double timeout=3;
        double distance=5;
        double tolerance=2.0;
        double heading=0;

        System.out.println("HERE!");

        checkBail(waitForInput);
        driveBrain.rotateToHeadingAbsolute(0, tolerance, speed, timeout);
        stage="demoDriveDistance - distance!";
        checkBail(waitForInput);
        driveBrain.driveDistance(distance, speed, timeout);
        stage="demoDriveDistance - rotate";
        checkBail(waitForInput);
        driveBrain.rotateToHeadingAbsolute(90, tolerance, speed, timeout);
        stage="demoDriveDistance - distance!";
        checkBail(waitForInput);
        driveBrain.driveDistance(distance, speed, timeout);
        stage="demoDriveDistance - rotate";
        checkBail(waitForInput);
        driveBrain.rotateToHeadingAbsolute(-90, tolerance, speed, timeout);
        checkBail(waitForInput);
        driveBrain.driveDistance(distance, speed, timeout);
    }

    void demoArm(boolean waitForInput) throws BailException {
        if (robot.useArm) {
            stage="demoArm";
            checkBail(waitForInput);
            robot.armSetPosition(1.0);
            sleep(1000);
            checkBail(waitForInput);
            robot.armSetPosition(1.0);
            sleep(1000);
        }


    }

    private void checkBail() throws BailException {
        checkBail(false);
    }

    private void checkBail(boolean waitForInput) throws BailException {
        if (!opModeIsActive())
            throw new BailException("!opModeIsActive()");
        else if (isStopRequested())
            throw new BailException("isStopRequested()");
        if (waitForInput) waitForInput();
        telemetry.update();
    }
    void waitForInput() throws BailException {
        while (!gamepad1.a) {
            telemetry.addData("Status", "Press A for next Action.");
            telemetry.addData("Stage", stage);
            telemetry.update();
            sleep(1);
            checkBail(false);
        }
    }
}
