package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "XDriveMenuDriven", group = "7079")
@Disabled
public class MenuDriven extends LinearOpMode {
    faltechBotXDrive robotXDrive = new faltechBotXDrive();
    DriveBrainXDrive driveBrain;
    Utility Utility;
    double fixedHeading = 0;
    private ElapsedTime runtime = new ElapsedTime();
    OpMode opmode;
    XDriveVisionBrain visionXDrive;

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double     STRAFE_SPEED  = 0.5;
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

        robotXDrive.setDrive(FORWARD_SPEED, 0, 0,POWER);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            if (gamepad1.a) {
                robotXDrive.setDrive(5, 0, 90, POWER);
            }

        }
        robotXDrive.setDriveStop();
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
