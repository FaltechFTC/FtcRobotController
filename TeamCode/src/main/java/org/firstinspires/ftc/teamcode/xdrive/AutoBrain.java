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
    public static double TEST_CONFIG = 1.0;
    public static double LONG_MAINT_SECS = 1.4;
    RobotDrive robot;
    DriveBrain driveBrain;
    VisionBrain vision;
    boolean useVision = false;
    Telemetry telemetry;
    ElapsedTime runtime = new ElapsedTime();
    double fixedHeading = 0;
    double verySlowPower = 0.1;
    double slowPower = 0.2;
    double mediumPower = 0.3;
    double halfPower = 0.5;
    double highPower = 0.7;
    double veryHighPower = 1;
    double shortTimeout = 2;
    double mediumTimeout = 3;
    double highTimeout = 5;
    int armMarkerPos = 0;

    public void init(LinearOpMode opmode) {
        telemetry = opmode.telemetry;
//        telemetry.addData("Status", "RunOpMode");
//        telemetry.update();

        robot = new RobotDrive();
        robot.init(opmode.hardwareMap, telemetry);
        robot.setDriveStopModeBreak();
        robot.intake.maxUpPower = .3;// slower during auto
        robot.intake.magnetEngage();
        //robot.wristMove(0);
//        telemetry.addData("Status", "Robot Initialized");
//        telemetry.update();

        driveBrain = new DriveBrain(robot, opmode);
        robot.intake.maxUpPower = .3; // slower during auto
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
        } while (setZeroTimer.seconds() < 3.0
                && (Math.abs(heading) < 0.1
                || Math.abs(heading - prev_heading) > 1.));
        robot.setZeroHeading();
        telemetry.addData("Heading", robot.getHeading(AngleUnit.DEGREES));
        telemetry.update();

    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
            // eat it
        }
    }

    public void autoNothing() throws Exception {
        telemetry.addData("Status", "Doing Nothing");
        telemetry.update();
        sleep(2000);
        telemetry.addData("Status", "Done");
        telemetry.update();
        sleep(1000);
    }

    public void autoPosTest() {
        driveBrain.driveDistance(20, mediumPower, 1);
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(90, 3, mediumPower, 3);
        sleep(5000);
        driveBrain.driveDistance(20, mediumPower, 3);//drives to shipping hub
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(180, 3, mediumPower, 3);//rotates so that it is facing carousel
        sleep(5000);
        driveBrain.carouselMoves(1);//moves the carousel wheel
        sleep(5000);
        driveBrain.driveDistance(20, mediumPower, 3);//drives to shipping hub
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(270, 3, mediumPower, 2);
        sleep(5000);
        driveBrain.driveDistance(20, mediumPower, 3);//drives to carousel
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(360, 3, mediumPower, 5);
    }

    //carousel = false then we don't do carousel, else do carousel

    public void autoPark1Red() {

        int barcode = 0; //TODO get barcode number from vision
        //double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        robot.intake.setGantryPosition(Robot.ARM_LAYER3_POS - 15,0);//moves arm straight up
        driveBrain.maintTime(.250);
//
        driveBrain.driveDistance(rp1_d1 / 2, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(rp1_d1 / 2, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-15, 2, mediumPower, mediumTimeout);

        robot.intake.wristMove(0);
        robot.intake.magnetRelease();
        driveBrain.maintTime(rp1_m2);

        robot.intake.setGantryPosition(Robot.ARM_PARK_POS,0);

//        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
//        driveBrain.driveDistance(-rp1_d1/2, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance((-rp1_d1) + 1, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-60, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(-35, mediumPower, 3);

        robot.setDrive(-.2, -.1, 0, verySlowPower);
        robot.intake.carousel.setPower(.35);
        driveBrain.maintTime(5);
        robot.intake.carousel.setPower(0);
        robot.setDriveStop();

        driveBrain.rotateToHeadingAbsolute(45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(21, mediumPower, mediumTimeout);
        driveBrain.rotateToHeadingAbsolute(0, 3, mediumPower, mediumTimeout);
//        driveBrain.setArmMotorPosition(Robot.ARM_INTAKE_POS);
        robot.intake.magnetEngage();
        driveBrain.maintTime(.5);
    }

    public void autoPark1Blue() {
        int barcode = 0; //TODO get barcode number from vision
        //double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        robot.intake.setGantryPosition(Robot.ARM_LAYER3_POS - 15,0);//moves arm straight up
        driveBrain.maintTime(.250);
//
        driveBrain.rotateToHeadingAbsolute(60, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(rp1_d1 + 2, mediumPower, 3);
//        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
//        driveBrain.driveDistance(rp1_d1/2, mediumPower, 3);
//        driveBrain.rotateToHeadingAbsolute(-15, 2, mediumPower, mediumTimeout);

//        robot.wristMove(0);
        robot.intake.magnetRelease();
        driveBrain.maintTime(rp1_m2);

        robot.intake.setGantryPosition(Robot.ARM_PARK_POS,0);
//easter egg//        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
//        driveBrain.driveDistance(-rp1_d1/2, mediumPower, 3);
//        driveBrain.rotateToHeadingAbsolute(15, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance((-rp1_d1) + 1, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-20, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(35, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(45, 3, mediumPower, mediumTimeout);

        robot.setDrive(-.2, -.1, 0, verySlowPower);
        robot.intake.carousel.setPower(-.35);
        driveBrain.maintTime(5);
        robot.intake.carousel.setPower(0);
        robot.setDriveStop();

//        driveBrain.rotateToHeadingAbsolute(45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(21, mediumPower, mediumTimeout);
        driveBrain.rotateToHeadingAbsolute(0, 3, mediumPower, mediumTimeout);
//        driveBrain.setArmMotorPosition(Robot.ARM_INTAKE_POS);
        robot.intake.magnetEngage();
        driveBrain.maintTime(.5);
    }

    public void autoPark2Red() {
        int barcode = 0; //TODO get barcode number from vision
        //double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        robot.intake.setGantryPosition(Robot.ARM_LAYER3_POS,0);//moves arm straight up
        driveBrain.maintTime(.250);
//easter egg
        driveBrain.rotateToHeadingAbsolute(90, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(rp1_d1, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(0, 3, mediumPower, mediumTimeout);

        robot.intake.wristMove(0);
        robot.intake.magnetRelease();
        driveBrain.maintTime(rp1_m2);

        robot.intake.setGantryPosition(Robot.ARM_PARK_POS,0);

        driveBrain.rotateToHeadingAbsolute(90, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(-rp1_d1 + .5, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(30, mediumPower, 3);

        driveBrain.rotateToHeadingAbsolute(45, 2, mediumPower, mediumTimeout);
        driveBrain.driveDistance(16, mediumPower, mediumTimeout);
        driveBrain.rotateToHeadingAbsolute(-135, 3, mediumPower, mediumTimeout);
        robot.intake.setGantryPosition(Robot.ARM_INTAKE_POS,0);
        robot.intake.magnetEngage();
        driveBrain.maintTime(1);
    }

    public void autoPark2Blue() {
        int barcode = 0; //TODO get barcode number from vision
        //double distance = robot.distanceSensor.getDistance(DistanceUnit.INCH);
        robot.intake.setGantryPosition(Robot.ARM_LAYER3_POS,0);//moves arm straight up
        driveBrain.maintTime(.250);

//        driveBrain.rotateToHeadingAbsolute(90, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(rp1_d1, mediumPower, 3);

        robot.intake.magnetRelease();
        driveBrain.maintTime(rp1_m2);

        robot.intake.setGantryPosition(Robot.ARM_PARK_POS,0);

        driveBrain.driveDistance(-rp1_d1 + .5, mediumPower, 3);
        driveBrain.rotateToHeadingAbsolute(-45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(-35, mediumPower, 3);

        driveBrain.rotateToHeadingAbsolute(45, 3, mediumPower, mediumTimeout);
        driveBrain.driveDistance(25, mediumPower, mediumTimeout);
        driveBrain.rotateToHeadingAbsolute(135, 3, halfPower, mediumTimeout);
        robot.intake.setGantryPosition(Robot.ARM_INTAKE_POS,0);
        robot.intake.magnetEngage();
        driveBrain.maintTime(1);
    }
}
