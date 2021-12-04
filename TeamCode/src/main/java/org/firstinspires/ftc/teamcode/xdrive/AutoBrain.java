package org.firstinspires.ftc.teamcode.xdrive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rp1_d1;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rp1_m2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class AutoBrain {
    Robot robot;
    DriveBrain driveBrain;
    VisionBrain vision;
    boolean useVision=false;
    Telemetry telemetry;

    ElapsedTime runtime = new ElapsedTime();

    double fixedHeading = 0;
    double verySlowPower = 0.1;
    double slowPower=0.2;
    double mediumPower=0.3;
    double halfPower = 0.5;
    double highPower = 0.7;
    double veryHighPower = 1;
    double shortTimeout=2;
    double mediumTimeout=3;
    double highTimeout = 5;
    int armMarkerPos = 0;

    public static double TEST_CONFIG = 1.0;

    public void init(LinearOpMode opmode) {
        telemetry = opmode.telemetry;
//        telemetry.addData("Status", "RunOpMode");
//        telemetry.update();

        robot = new Robot();
        robot.init(opmode.hardwareMap, telemetry);
        robot.setDriveStopModeBreak();
        robot.maxUpPower=.3;// slower during auto
        robot.magnetEngage();
        //robot.wristMove(0);
//        telemetry.addData("Status", "Robot Initialized");
//        telemetry.update();

        driveBrain = new DriveBrain(robot, opmode);
        robot.maxUpPower=.3; // slower during auto
//        telemetry.addData("Status", "DriveBrain Ready");
//        telemetry.update();

        if (useVision) {

            vision = new VisionBrain();
            vision.showCamera = true; // useful for sighting on phone only
            vision.showCameraOD = false; // useful for seeing object detection on phone only
            vision.zoom = 1f;  // 1.0 is no zoom, greater number is greater zoom
            vision.init(opmode);
//            telemetry.addData("Status", "Vision Ready");
//            telemetry.update();

            // vision.activate();
        }

        ElapsedTime setZeroTimer = new ElapsedTime();
        setZeroTimer.reset();

        // wait for heading to be non-zero and stable
        double heading = 0.;
        double prev_heading = 0.;
        do {
            sleep(100);
            prev_heading = heading;
            heading = robot.getRawHeading(AngleUnit.DEGREES);
        } while(setZeroTimer.seconds() < 3.0
                && (Math.abs(heading) < 0.1
                || Math.abs(heading - prev_heading) > 1.));
        robot.setZeroHeading();
        telemetry.addData("Heading", robot.getHeading(AngleUnit.DEGREES));
        telemetry.update();

    }

    public void sleep (int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
            ; // eat it
        }
    }

     public void autoNothing() throws Exception{
         telemetry.addData("Status", "Doing Nothing");
         telemetry.update();
         sleep(2000);
         telemetry.addData("Status", "Done");
         telemetry.update();
         sleep(1000);
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
        driveBrain.carouselMoves(1);//moves the carousel wheel
        sleep(5000);
        driveBrain.driveDistance(20, mediumPower,3);//drives to shipping hub
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(270,3,mediumPower,2);
        sleep(5000);
        driveBrain.driveDistance(20, mediumPower, 3);//drives to carousel
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(360,3,mediumPower,5);
    }

    public void autoPosScore1() {
        //double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);//moves arm straight up
        driveBrain.rotateToHeadingAbsolute(75, 2, slowPower, shortTimeout);
        driveBrain.driveDistance(27, slowPower, 3);
        driveBrain.rotateToHeadingAbsolute(-15, 2, slowPower, shortTimeout);
        driveBrain.setArmMotorPosition(Robot.ARM_LAYER3_POS);
        robot.setWristOffset(.53);
        driveBrain.pusherStart(2000);
        driveBrain.maintTime(1);
        driveBrain.rotateToHeadingAbsolute(110,2, slowPower, shortTimeout);
        driveBrain.driveDistance(36, slowPower, shortTimeout);
        driveBrain.carouselStart(false);
        driveBrain.carouselMaint();
        driveBrain.rotateToHeadingAbsolute(0, 2, slowPower, shortTimeout);
        driveBrain.driveDistance(24, slowPower, shortTimeout);
    }
    public void autoPosScore2() {

        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);
        driveBrain.rotateToHeadingAbsolute(45,2, mediumPower, shortTimeout);
        driveBrain.driveDistance(24, mediumPower, shortTimeout);
        driveBrain.rotateToHeadingAbsolute(-50, 2, halfPower, shortTimeout);
        driveBrain.driveDistance(24, slowPower, shortTimeout);
        robot.setWristOffset(.53);
        driveBrain.pusherStart(2000);
        driveBrain.maintTime(1);
    }

    //carousel = false then we don't do carousel, else do carousel

    public static double LONG_MAINT_SECS=1.4;

    public void simpleTest() {
        if (false) {
            driveBrain.setArmMotorPosition(Robot.ARM_LAYER3_POS);
            driveBrain.maintTime(LONG_MAINT_SECS);
            driveBrain.setArmMotorPosition(Robot.ARM_LAYER2_POS);
            driveBrain.maintTime(LONG_MAINT_SECS);
            driveBrain.setArmMotorPosition(Robot.ARM_LAYER1_POS);
            driveBrain.maintTime(LONG_MAINT_SECS);
        }

        telemetry.addData("current heading", robot.getHeading(AngleUnit.DEGREES));
        telemetry.addData("raw heading", robot.getRawHeading(AngleUnit.DEGREES));
        sleep(3000);

        if (true) {
            driveBrain.setArmMotorPosition(Robot.ARM_INTAKE_POS);
            driveBrain.rotateToHeadingAbsolute(0, 2, mediumPower, mediumTimeout);
            driveBrain.rotateToHeadingAbsolute(90, 2, mediumPower, mediumTimeout);
            driveBrain.rotateToHeadingAbsolute(-90, 2, mediumPower, mediumTimeout);
        }

        if (true) {
            robot.wristMove(0);
            driveBrain.maintTime(LONG_MAINT_SECS);
            robot.wristMove(.5);
            driveBrain.maintTime(LONG_MAINT_SECS);

            robot.magnetRelease();
            driveBrain.maintTime(1);
            robot.magnetEngage();
            driveBrain.maintTime(1);
        }

    }

    public void autoPark1Red() {

        int barcode = 0; //TODO get barcode number from vision
        //double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        driveBrain.setArmMotorPosition(Robot.ARM_LAYER3_POS);//moves arm straight up
        driveBrain.maintTime(.250);
//
        driveBrain.driveDistance(rp1_d1/2, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(rp1_d1/2, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-15, 2, mediumPower, mediumTimeout);

        robot.wristMove(0);
        robot.magnetRelease();
        driveBrain.maintTime(rp1_m2);

        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);

//        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
//        driveBrain.driveDistance(-rp1_d1/2, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance((-rp1_d1)+1, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-60, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(-35, mediumPower, 3);

        robot.setDrive(-.2,-.1,0,verySlowPower);
        robot.carousel.setPower(.35);
        driveBrain.maintTime(5);
        robot.carousel.setPower(0);
        robot.setDriveStop();

        driveBrain.rotateToHeadingAbsolute(45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(23, mediumPower, mediumTimeout);
        driveBrain.rotateToHeadingAbsolute(0, 3, mediumPower, mediumTimeout);
        driveBrain.setArmMotorPosition(Robot.ARM_INTAKE_POS);
        robot.magnetEngage();
        driveBrain.maintTime(.5);
    }

    public void autoPark1Blue() {
        int barcode = 0; //TODO get barcode number from vision
        //double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        driveBrain.setArmMotorPosition(Robot.ARM_LAYER3_POS);//moves arm straight up
        driveBrain.maintTime(.250);
//
        driveBrain.rotateToHeadingAbsolute(60, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(rp1_d1+2, mediumPower, 3);
//        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
//        driveBrain.driveDistance(rp1_d1/2, mediumPower, 3);
//        driveBrain.rotateToHeadingAbsolute(-15, 2, mediumPower, mediumTimeout);

//        robot.wristMove(0);
        robot.magnetRelease();
        driveBrain.maintTime(rp1_m2);

        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);
//easter egg//        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
//        driveBrain.driveDistance(-rp1_d1/2, mediumPower, 3);
//        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance((-rp1_d1)+1, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-20, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(35, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(45, 3, mediumPower, mediumTimeout);

        robot.setDrive(-.2,-.1,0,verySlowPower);
        robot.carousel.setPower(-.35);
        driveBrain.maintTime(5);
        robot.carousel.setPower(0);
        robot.setDriveStop();

//        driveBrain.rotateToHeadingAbsolute(45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(23, mediumPower, mediumTimeout);
        driveBrain.rotateToHeadingAbsolute(0, 3, mediumPower, mediumTimeout);
        driveBrain.setArmMotorPosition(Robot.ARM_INTAKE_POS);
        robot.magnetEngage();
        driveBrain.maintTime(.5);
    }

    public void autoPark1BlueOG(boolean sideBlue, boolean doCarousel) {
        int barcode = 0; //TODO get barcode number from vision
        int modifier = 1;
        if (sideBlue) modifier = -1;
        telemetry.addData("Starting!", 0);
        //deliver preloaded box
        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);//moves arm straight up
        driveBrain.maintTime(1.5);
        driveBrain.driveDistance(-20.5*modifier, mediumPower, mediumTimeout);

        if (doCarousel) {
            driveBrain.rotateToHeadingAbsolute(-90*modifier, 3, halfPower, mediumTimeout);
            robot.setDrive(-.2, .1, 0, 1);
            sleep(150);
            driveBrain.carouselMoves(-1);//moves the carousel wheel
            robot.setDriveStop();
        }
        driveBrain.rotateToHeadingAbsolute(-80 * modifier, 3, 0.5, mediumTimeout);
        driveBrain.driveDistance(-27*modifier, mediumPower, 2);
//        driveBrain.rotateToHeadingAbsolute(sideBlue?180:0, 3, 0.5, mediumTimeout);

        driveBrain.setArmMotorPosition(0);
        driveBrain.maintTime(1.5);

    }
    //carousel = false then we don't do carousel, else do carousel
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
            driveBrain.carouselMoves(1*angleModifier);//moves the carousel wheel
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
//        robot.magnetEngage();
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
    public void autoPark2Red(){
        int barcode = 0; //TODO get barcode number from vision
        //double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        driveBrain.setArmMotorPosition(Robot.ARM_LAYER3_POS);//moves arm straight up
        driveBrain.maintTime(.250);
//easter egg
        driveBrain.rotateToHeadingAbsolute(90, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(rp1_d1, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(0, 3, mediumPower, mediumTimeout);

        robot.wristMove(0);
        robot.magnetRelease();
        driveBrain.maintTime(rp1_m2);

        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);

        driveBrain.rotateToHeadingAbsolute(90, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(-rp1_d1+.5, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(30, mediumPower, 3);

        driveBrain.rotateToHeadingAbsolute(45, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(16, mediumPower, mediumTimeout);
        driveBrain.rotateToHeadingAbsolute(-135, 3, mediumPower, mediumTimeout);
        driveBrain.setArmMotorPosition(Robot.ARM_INTAKE_POS);
        robot.magnetEngage();
        driveBrain.maintTime(1);
    }

    public void autoPark2Blue() {
        int barcode = 0; //TODO get barcode number from vision
        //double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        driveBrain.setArmMotorPosition(Robot.ARM_LAYER3_POS);//moves arm straight up
        driveBrain.maintTime(.250);

//        driveBrain.rotateToHeadingAbsolute(90, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(rp1_d1, mediumPower, 3);

        robot.magnetRelease();
        driveBrain.maintTime(rp1_m2);

        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);

        driveBrain.driveDistance(-rp1_d1+.5, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(-35, mediumPower, 3);

        driveBrain.rotateToHeadingAbsolute(45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(25, mediumPower, mediumTimeout);
        driveBrain.rotateToHeadingAbsolute(135, 3, halfPower, mediumTimeout);
        driveBrain.setArmMotorPosition(Robot.ARM_INTAKE_POS);
        robot.magnetEngage();
        driveBrain.maintTime(1);
    }

    public void autoPark2BlueOG(boolean sideBlue) {
        int barcode = 0; //TODO get barcode number from vision
        int modifier = 1;
        if (sideBlue) modifier=-1;

        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);//moves arm straight up
        driveBrain.maintTime(1.5);
        driveBrain.driveDistance(34*modifier,mediumPower, highTimeout);
        driveBrain.rotateToHeadingAbsolute(0, 4, slowPower, shortTimeout);
        if (false) { // move out the way
            driveBrain.rotateToHeadingAbsolute(45*modifier,4,0.25,mediumTimeout);
            driveBrain.driveDistance(25*modifier,mediumPower, mediumTimeout);//drives to warehouse
            driveBrain.setArmMotorPosition(Robot.ARM_INTAKE_POS);
            driveBrain.maintTime(1.5);
        } else {
            driveBrain.setArmMotorPosition(100);
            // TODO set wrist!!
            driveBrain.maintTime(1.5);
            driveBrain.driveDistance(-6*modifier,mediumPower, shortTimeout);
//            driveBrain.rotateToHeadingAbsolute(-45 * modifier, 4, 0.35, shortTimeout);
            driveBrain.rotateToHeadingAbsolute(-90 * modifier, 4, 0.35, mediumTimeout);
            driveBrain.driveDistance(-20*modifier, .25, mediumTimeout);//drives to warehouse
        }
    }
//easteregg
    public void autoPark2RedOG(boolean sideBlue) {
        int barcode = 0; //TODO get barcode number from vision
        int modifier = 1;
        if (sideBlue) modifier=-1;

        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);
        driveBrain.maintTime(1.5);
        driveBrain.driveDistance(34*modifier,mediumPower, highTimeout);
        driveBrain.rotateToHeadingAbsolute(0, 4, slowPower, shortTimeout);
        if (false) { // move out the way
            driveBrain.rotateToHeadingAbsolute(45*modifier,4,0.25,mediumTimeout);
            driveBrain.driveDistance(25*modifier,mediumPower, mediumTimeout);
            driveBrain.setArmMotorPosition(Robot.ARM_INTAKE_POS);
            driveBrain.maintTime(1.5);
        } else {
            driveBrain.setArmMotorPosition(100);
            // TODO set wrist!!
            driveBrain.maintTime(1.5);
            driveBrain.driveDistance(-6*modifier,mediumPower, shortTimeout);
//            driveBrain.rotateToHeadingAbsolute(-45 * modifier, 4, 0.35, shortTimeout);
            driveBrain.rotateToHeadingAbsolute(-90 * modifier, 4, 0.35, mediumTimeout);
            driveBrain.driveDistance(-20*modifier, .25, mediumTimeout);//drives to warehouse
//
        }
    }
}
