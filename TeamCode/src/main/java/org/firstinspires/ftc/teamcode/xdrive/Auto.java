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
//        driveBrain.setZeroHeading();
       // autoPos1(false);
        autoPosTest();
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
        driveBrain.driveDistance(1,mediumPower, shortTimeout);//move forward 1in
        driveBrain.rotateToHeadingAbsolute(-30*angleModifier,2,0.5,mediumTimeout);
        driveBrain.driveDistance(34,halfPower,mediumTimeout);//drives to shipping hub
        telemetry.addData("Moved to shipping hub!",1);
        //TODO move the arm so that we can place the game piece
        telemetry.addData("Placed Block!",2);
        //gets duck off of carousel
        driveBrain.driveDistance(1,halfPower,shortTimeout);//drives 1 inch in order to get enough room to spin
        driveBrain.rotateToHeadingAbsolute(110*angleModifier,3,halfPower,mediumTimeout);//rotates so that it is facing carousel
        driveBrain.driveDistance(68, highPower, mediumTimeout);//drives to carousel
        driveBrain.carouselMoves();//moves the carousel wheel
        telemetry.addData("Moved the Carousel",3);
        //Gets a block from the warehouse and delivers it
        driveBrain.driveDistance(-1, mediumTimeout, shortTimeout);//goes back so that it has room to turn
        driveBrain.rotateToHeadingAbsolute(270*angleModifier,2,mediumPower,highTimeout);
        driveBrain.driveDistance(96,halfPower, highTimeout);//drives to warehouse
        telemetry.addData("Drove to Warehouse!",4);
        //TODO make it so that the arm and claw work together to get a block.
        driveBrain.driveDistance(-72,halfPower,highTimeout);

        driveBrain.rotateToHeadingAbsolute(360*angleModifier,2,halfPower,highTimeout);
        driveBrain.driveDistance(33, highPower,highTimeout);//drives to shipping hub
        telemetry.addData("Drove to Shipping Hub!",5);
        //TODO places block down
        telemetry.addData("Placed Block!",6);
        //Parks in the warehouse
        driveBrain.driveDistance(-33, slowPower, shortTimeout);//goes back so that it has enough room to turn
        driveBrain.rotateToHeadingAbsolute(270*angleModifier,2,halfPower,mediumTimeout);
        driveBrain.driveDistance(72, halfPower, mediumTimeout);//drives to go inside the warehouse
        telemetry.addData("Drove to Warehouse!",7);
        //end
        telemetry.addData("Autonomous For Resource Depot Complete!",8);
    }
    public void autoPos2(boolean side) {
        int barcode = 0; //TODO get barcode number from vision
        int angleModifier = 1;
        if (side) angleModifier=-1;
    }
    public void autoPosTest(){
//        driveBrain.driveDistance(20,mediumPower, 1);
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(90,3,mediumPower,3);
        sleep(5000);
//        driveBrain.driveDistance(20,mediumPower,3);//drives to shipping hub
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(180,3,mediumPower,3);//rotates so that it is facing carousel
        sleep(5000);
        driveBrain.carouselMoves();//moves the carousel wheel
        sleep(5000);
//        driveBrain.driveDistance(20, mediumPower,3);//drives to shipping hub
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(270,3,mediumPower,2);
        sleep(5000);
//        driveBrain.driveDistance(20, mediumPower, 3);//drives to carousel
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(360,3,mediumPower,5);
    }
}
