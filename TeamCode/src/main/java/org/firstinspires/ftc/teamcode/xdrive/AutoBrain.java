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
        robot.intake.magnetEngage();
        //robot.wristMove(0);
//        telemetry.addData("Status", "Robot Initialized");
//        telemetry.update();

        driveBrain = new DriveBrain(robot, opmode);
//        telemetry.addData("Status", "DriveBrain Ready");
//        telemetry.update();

        if (useVision) {

            vision = new VisionBrain();
            vision.showCamera = true; // useful for sighting on phone only
            vision.showCameraOD = false; // useful for seeing object detection on phone only
            vision.zoom = 1f;  // 1.0 is no zoom, greater number is greater zoom
            vision.init(opmode,telemetry);
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









}
