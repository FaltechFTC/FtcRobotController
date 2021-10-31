package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", group = "7079")
public class Auto extends LinearOpMode {
    Robot robot = new Robot();
    DriveBrain driveBrain;
    double fixedHeading = 0;
    private ElapsedTime runtime = new ElapsedTime();

    VisionBrain visionDrive;

//    static final double     FORWARD_SPEED = 0.6;
//    static final double     TURN_SPEED    = 0.5;
//    static final double     STRAFE_SPEED  = 0.5;
    static final double     POWER         = 1.0;

    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);
        robot.setDriveStopModeBreak();
        driveBrain = new DriveBrain(robot, this);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        driveBrain.setZeroHeading();
        autoPos1(false);
        
        robot.setDriveStop();
    }
    //Side = true is blue
    //Side = false is red
    public void autoPos1(boolean side) {
        int barcode = 0; //TODO get barcode number from vision
        int angleModifier = 1;
        if (side) angleModifier=-1;

        //deliver preloaded box
        driveBrain.driveDistance(1,.3, 1);//move forward 1in
        robot.setDriveStop();
        robot.setDrive(0,0,-30*angleModifier,POWER);//rotates 30degrees to go to the shipping hub
        robot.setDriveStop();
        driveBrain.driveDistance(34,.5,2);//drives to shipping hub
        //TODO move the arm so that we can place the game piece

        //gets duck off of carousel
        driveBrain.driveDistance(1,.5,1);//drives 1 inch in order to get enough room to spin
        robot.setDriveStop();
        robot.setDrive(0,0, 140*angleModifier,POWER);//rotates so that it is facing carousel
        robot.setDriveStop();
        driveBrain.driveDistance(68, .75, 3);//drives to carousel
        driveBrain.carouselMoves();//moves the carousel wheel

        //Gets a block from the warehouse and delivers it
        driveBrain.driveDistance(-1, .3, 1);//goes back so that it has room to turn
        robot.setDriveStop();
        robot.setDrive(0,0,160*angleModifier, .3);//turns so that it is facing the warehouse
        robot.setDriveStop();
        driveBrain.driveDistance(96,.5, 6);//drives to warehouse
        //TODO make it so that the arm and claw work together to get a block.
        driveBrain.driveDistance(-72,.5,5);
        robot.setDriveStop();
        robot.setDrive(0,0,90*angleModifier,.5);//rotates so that it is facing the outside of the warehouse
        robot.setDriveStop();
        driveBrain.driveDistance(33, .6,5);//drives to shipping hub
        //TODO places block down

        //Parks in the warehouse
        driveBrain.driveDistance(-33, .2, 1);//goes back so that it has enough room to turn
        robot.setDriveStop();
        robot.setDrive(0,0,-90*angleModifier,.5);
        robot.setDriveStop();
        driveBrain.driveDistance(72, .5, 3);//drives to go inside the warehouse
        //end

    }
    public void redAutoWarehouse(boolean side) {
        int barcode = 0; //TODO get barcode number from vision
        int angleModifier = 1;
        if (side) angleModifier=-1;
    }
}
