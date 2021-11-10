package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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
    int armMarkerPos = 0;
    private ElapsedTime runtime = new ElapsedTime();

    VisionBrain vision;
    boolean useVision=false;
    static final double     POWER         = 1.0;


    public void runOpMode() {

        telemetry.addData("Status", "RunOpMode");    //
        telemetry.update();


        robot.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Robot Initialized");    //
        telemetry.update();

        robot.setDriveStopModeBreak();
        driveBrain = new DriveBrain(robot, this);
        telemetry.addData("Status", "DriveBrain Ready");    //
        telemetry.update();

        if (useVision) {
            vision = new VisionBrain();
            vision.showCamera = true; // useful for sighting on phone only
            vision.showCameraOD = false; // useful for seeing object detection on phone only
            vision.zoom = 1f;  // 1.0 is no zoom, greater number is greater zoom
            vision.init(this);
            //        vision.activate();
        }
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //doMission();
        autoPos1(false, true);

        robot.setDriveStop();
    }

        public  void doMission() {}

    @Autonomous(name = "Auto Red1Std", group = "7079")
    public class Red1Std extends Auto {
        public Red1Std()
        {
            super();
        }

        public void doMission()
        {
            autoPos1(false, true);
        }

    }

    @Autonomous(name = "Auto Red2Std", group = "7079")
    public class Red2Std extends Auto {
        public Red2Std() {
            super();
        }
        public void Red2Std()
        {
            autoPos1(false, true);
        }

    }
        //Side = true is blue
    //Side = false is red
    //carousel = false then we don't do carousel
    //carousel = true then we do carousel
    public void autoPos1(boolean sideBlue, boolean doCarousel) {
        int barcode = 0; //TODO get barcode number from vision
        int angleModifier = 1;
        if (sideBlue) angleModifier=-1;
        telemetry.addData("Starting!",0);
        //deliver preloaded box
        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);//moves arm straight up
        driveBrain.maintTime(1.5);
        driveBrain.driveDistance(1,mediumPower, shortTimeout);//move forward 1in
        driveBrain.rotateToHeadingAbsolute(-30*angleModifier,2,0.5,mediumTimeout);
        driveBrain.driveDistance(30,halfPower,mediumTimeout);//drives to shipping hub
        telemetry.addData("Moved to shipping hub!",1);
        if (barcode==1) armMarkerPos = Robot.ARM_LAYER1_POS;
        if (barcode==2) armMarkerPos = Robot.ARM_LAYER2_POS;
        if (barcode==3) armMarkerPos = Robot.ARM_LAYER3_POS;
        robot.setArmMotorPosition(armMarkerPos);//TODO make it so that when the barcode is read then it goes to that level
        telemetry.addData("Placed Block!",2);
        //drives to warehouse
        if (doCarousel) {
            //gets duck off of carousel
            driveBrain.driveDistance(1,halfPower,shortTimeout);//drives 1 inch in order to get enough room to spin
            driveBrain.rotateToHeadingAbsolute(110*angleModifier,3,halfPower,mediumTimeout);//rotates so that it is facing carousel
            driveBrain.driveDistance(68, highPower, mediumTimeout);//drives to carousel
            driveBrain.carouselMoves();//moves the carousel wheel
            telemetry.addData("Moved the Carousel",3);
//          Gets a block from the warehouse and delivers it
            driveBrain.driveDistance(-1, mediumTimeout, shortTimeout);//goes back so that it has room to turn
            driveBrain.rotateToHeadingAbsolute(270*angleModifier,2,mediumPower,highTimeout);
            driveBrain.driveDistance(96,halfPower, highTimeout);//drives to warehouse
        }
        else {
            driveBrain.driveDistance(-1, mediumTimeout, shortTimeout);//goes back so that it has room to turn
            driveBrain.rotateToHeadingAbsolute(60*angleModifier,2,mediumPower,highTimeout);
            driveBrain.driveDistance(48,halfPower, highTimeout);//drives to warehouse
        }
        telemetry.addData("Drove to Warehouse!",4);
//        robot.pusherOpen();
//        driveBrain.setArmMotorPosition(128);
//        driveBrain.rotateToHeadingRelative(30, 1, mediumPower, shortTimeout);
//        driveBrain.setArmMotorPosition(353);
//        //TODO make it so that the arm and claw work together to get a block.
//        driveBrain.driveDistance(-72,halfPower,highTimeout);
//
//        driveBrain.rotateToHeadingAbsolute(360*angleModifier,2,halfPower,highTimeout);
//        driveBrain.driveDistance(33, highPower,highTimeout);//drives to shipping hub
//        telemetry.addData("Drove to Shipping Hub!",5);
//        robot.pusherClose();
//        telemetry.addData("Placed Block!",6);
//        //Parks in the warehouse
//        driveBrain.driveDistance(-33, slowPower, shortTimeout);//goes back so that it has enough room to turn
//        driveBrain.rotateToHeadingAbsolute(270*angleModifier,2,halfPower,mediumTimeout);
//        driveBrain.driveDistance(72, halfPower, mediumTimeout);//drives to go inside the warehouse
//        telemetry.addData("Drove to Warehouse!",7);
//        //end
//        telemetry.addData("Autonomous For Resource Depot Complete!",8);
    }
    public void autoPos2(boolean side) {
        int barcode = 0; //TODO get barcode number from vision
        int angleModifier = 1;
        if (side) angleModifier=-1;
        robot.setArmMotorPosition(750);
//        driveBrain.driveDistance();
    }
    public void autoPosTest(){
        driveBrain.driveDistance(20,mediumPower, 1);
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(90,3,mediumPower,3);
        sleep(5000);
        driveBrain.driveDistance(20,mediumPower,3);//drives to shipping hub
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(180,3,mediumPower,3);//rotates so that it is facing carousel
        sleep(5000);
        driveBrain.carouselMoves();//moves the carousel wheel
        sleep(5000);
        driveBrain.driveDistance(20, mediumPower,3);//drives to shipping hub
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(270,3,mediumPower,2);
        sleep(5000);
        driveBrain.driveDistance(20, mediumPower, 3);//drives to carousel
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(360,3,mediumPower,5);
    }



}
