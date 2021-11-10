package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name = "Auto Test", group = "7079")

public class AutoTester extends LinearOpMode {
    Robot robot = new Robot();
    DriveBrain driveBrain;
    double fixedHeading = 0;
    private ElapsedTime runtime = new ElapsedTime();
    OpMode opmode;
    VisionBrain visionXDrive;

//    static final double     FORWARD = 0.6;
//    static final double     TURN    = 0.5;
//    static final double     STRAFE  = 0.5;
    static final double     POWER         = 0.3;
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);
        driveBrain = new DriveBrain(robot, this);
        robot.setDriveStopModeBreak();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds


        runtime.reset();
        while (opModeIsActive()) {
            double drivepower1 = 0.5;
            if(gamepad1.left_bumper){
                if(gamepad1.a) driveSquare(10,.5,90);
                else if(gamepad1.b) figureEight(10,.5);
            }
           else if(gamepad1.a){
                driveBrain.rotateToHeadingAbsolute(40,3,1,20);
            }
            else if (gamepad1.b){
                driveBrain.rotateToHeadingRelative(20,5,1,25);
            }
           else if(gamepad1.y){
                driveBrain.driveDistance(5,drivepower1,10);
            }
           else if (gamepad1.dpad_up) {
                driveBrain.driveDistance(10, drivepower1, 10);
            }
           else if (gamepad1.dpad_down) {
                driveBrain.driveDistance(-10,drivepower1, 10);
                //robotXDrive.arm.setPosition(90);
            }

            else{
                robot.setDriveStop();
            }

        }
        robot.setDriveStop();
        telemetry.addData("Path", "Complete! :)");
        telemetry.update();
        sleep(1000);
    }
    public void driveSquare(double distance, double power, double angle){
        for(int i = 0;i<4;i++){
            driveBrain.driveDistance(distance,power,10);
            driveBrain.rotateToHeadingRelative(angle,3,power,10);
        }
    }
    public void figureEight(double distance, double power){
        driveSquare(distance,power,90);
        driveSquare(distance,power,-90);
    }

}
