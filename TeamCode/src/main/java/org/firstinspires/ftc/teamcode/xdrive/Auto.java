package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", group = "7079")
public class Auto extends LinearOpMode {
    Robot robot = new Robot();
    DriveBrain driveBrain;
    double fixedHeading = 0;
    double verySlowPower = 0.1;
    double slowPower=0.2;
    double mediumPower=0.3;
    double halfPower = 0.5;
    double highPower = 0.7;
    double veryHighPower = 1;
    double shortTimeout=1.5;
    double mediumTimeout=4;
    double highTimeout = 7;
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
        telemetry.addData("Starting!",0);
        //deliver preloaded box
        driveBrain.driveDistance(1,.3, 1);//move forward 1in

        driveBrain.rotateToHeadingAbsolute(-30,2,0.5,3);

        driveBrain.driveDistance(34,.5,2);//drives to shipping hub
        telemetry.addData("Moved to shipping hub!",1);
        //TODO move the arm so that we can place the game piece
        telemetry.addData("Placed Block!",2);
        //gets duck off of carousel
        driveBrain.driveDistance(1,.5,1);//drives 1 inch in order to get enough room to spin

        driveBrain.rotateToHeadingAbsolute(140,3,0.5,3);//rotates so that it is facing carousel

        driveBrain.driveDistance(68, .75, 3);//drives to carousel

        driveBrain.carouselMoves();//moves the carousel wheel
        telemetry.addData("Moved the Carousel",3);

        //Gets a block from the warehouse and delivers it
        driveBrain.driveDistance(-1, .3, 1);//goes back so that it has room to turn

        driveBrain.rotateToHeadingAbsolute(160,2,.3,5);

        driveBrain.driveDistance(96,.5, 6);//drives to warehouse
        telemetry.addData("Drove to Warehouse!",4);
        //TODO make it so that the arm and claw work together to get a block.
        driveBrain.driveDistance(-72,.5,5);

        driveBrain.rotateToHeadingAbsolute(90,2,.5,5);

        driveBrain.driveDistance(33, .6,5);//drives to shipping hub
        telemetry.addData("Drove to Shipping Hub!",5);
        //TODO places block down
        telemetry.addData("Placed Block!",6);
        //Parks in the warehouse
        driveBrain.driveDistance(-33, .2, 1);//goes back so that it has enough room to turn

        driveBrain.rotateToHeadingAbsolute(-90,2,.5,2);

        driveBrain.driveDistance(72, .5, 3);//drives to go inside the warehouse
        telemetry.addData("Drove to Warehouse!",7);
        //end
        telemetry.addData("Autonomous Complete!",8);
    }
    public void redAutoWarehouse(boolean side) {
        int barcode = 0; //TODO get barcode number from vision
        int angleModifier = 1;
        if (side) angleModifier=-1;
    }
}
