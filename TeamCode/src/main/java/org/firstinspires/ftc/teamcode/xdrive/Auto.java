package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", group = "7079")
@Disabled
public class Auto extends LinearOpMode {
    Robot robotXDrive = new Robot();
    DriveBrain driveBrain;
    org.firstinspires.ftc.teamcode.Utility Utility;
    double fixedHeading = 0;
    private ElapsedTime runtime = new ElapsedTime();
    OpMode opmode;
    VisionBrain visionXDrive;

//    static final double     FORWARD_SPEED = 0.6;
//    static final double     TURN_SPEED    = 0.5;
//    static final double     STRAFE_SPEED  = 0.5;
    static final double     POWER         = 1.0;

    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robotXDrive.init(robotXDrive.hwMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robotXDrive.setDriveStopModeBreak();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds

        robotXDrive.setDrive(5, 0, 0,POWER);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds

        robotXDrive.setDrive(0,0,90, POWER);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second
        robotXDrive.setDrive(-5, 0,0,POWER);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robotXDrive.setDrive(0,10,0,POWER);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robotXDrive.setDrive(0,-5,0,POWER);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            telemetry.addData("Path", "Leg 5: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        
        robotXDrive.setDriveStop();
        robotXDrive.arm.setPosition(1.0);
        robotXDrive.arm.setPosition(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
