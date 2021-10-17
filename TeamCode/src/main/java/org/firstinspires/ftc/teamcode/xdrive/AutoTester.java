package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Test", group = "7079")
@Disabled
public class AutoTester extends LinearOpMode {
    Robot robotXDrive = new Robot();
    DriveBrain driveBrain;
    org.firstinspires.ftc.teamcode.Utility Utility;
    double fixedHeading = 0;
    private ElapsedTime runtime = new ElapsedTime();
    OpMode opmode;
    VisionBrain visionXDrive;

//    static final double     FORWARD = 0.6;
//    static final double     TURN    = 0.5;
//    static final double     STRAFE  = 0.5;
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


        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            if (gamepad1.a) {
                robotXDrive.setDrive(5, 0, 90, POWER);
            }
            if (gamepad1.b) {
                robotXDrive.setDrive(10,28, 47, POWER);
//                robotXDrive.arm.setPosition(90);
            }

        }
        robotXDrive.setDriveStop();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
