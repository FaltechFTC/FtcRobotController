package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Tester", group = "7079")
//@Disabled
public class AutoTester extends LinearOpMode {
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
        } catch (Auto.BailException e) {
            telemetry.addData("Bailed!", e.getMessage());
            telemetry.update();
        } finally {
            robot.setDriveStop();
        }
    }

    private void runMission() throws Auto.BailException {
        stage="runMission";

        while (opModeIsActive()) {
            double speed=.3;
            double timeout=5.5;
            double distance=20;
            double tolerance=2.0;
            double heading=0;

            if (Math.abs(gamepad1.left_stick_x)>.05) robot.setDrive(0,gamepad1.left_stick_x,0);
            else if (Math.abs(gamepad1.left_stick_y)>.05) robot.setDrive(-gamepad1.left_stick_y,0,0);
            else if (gamepad1.right_stick_y>.1) driveBrain.rotateToHeadingAbsolute(0, 3, speed, timeout);
            else if (gamepad1.right_stick_y<-.1) driveBrain.rotateToHeadingAbsolute(180, 3, speed, timeout);
            else if (gamepad1.right_stick_x>.1) driveBrain.rotateToHeadingRelative(-90, 3, speed, timeout);
            else if (gamepad1.right_stick_x<-.1) driveBrain.rotateToHeadingRelative(90, 3, speed, timeout);
            else if (gamepad1.y) driveBrain.rotateToHeadingAbsolute(180, tolerance, speed, timeout);
            else if (gamepad1.b) driveBrain.driveDistance(distance, speed, timeout);
            else if (gamepad1.a) driveBrain.driveDistance(-distance, speed, timeout);
            else robot.setDriveStop();

        }

        robot.setDriveStop();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
