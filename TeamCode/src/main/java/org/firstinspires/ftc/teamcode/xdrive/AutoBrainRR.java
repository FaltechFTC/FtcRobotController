package org.firstinspires.ftc.teamcode.xdrive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Pose;

@Config
public class AutoBrainRR {
    public static double DRIFT_YPOW = .11;
    public static double DRIFT_XPOW = -.16;
    public static double TEST_CONFIG = 1.0;
    RobotRRDrive drive;
    VisionBrain vision;
    static public boolean useVision = false;
    Telemetry telemetry;
    LinearOpMode opmode;
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
    public static boolean doWarehousePark = true;

    public void init(LinearOpMode opmode) {
        this.opmode = opmode;
        telemetry = opmode.telemetry;
        drive = new RobotRRDrive(opmode.hardwareMap, opmode.telemetry);

        if (useVision) {

            vision = new VisionBrain();
            vision.useWebCam = true;
            vision.showCamera = false; // useful for sighting on phone only
            vision.showCameraOD = true; // useful for seeing object detection on phone only
            vision.zoom = 1f;  // 1.0 is no zoom, greater number is greater zoom
            vision.init(opmode, telemetry);
            vision.activate();
//            telemetry.addData("Status", "Vision Ready");
//            telemetry.update();

        }
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
            // eat it
        }
    }

    public void red1() throws InterruptedException {
        Pose2d startPose = new Pose2d(-36, 60.4, Math.toRadians(90));
        Pose2d sharedHubPose = new Pose2d(startPose.getX() + 15, startPose.getY() + 26, Math.toRadians(90 - 25));
        Pose2d midPoint = new Pose2d(startPose.getX() + 10, startPose.getY() + 20, Math.toRadians(90 - 25));
        Pose2d carouselPose = new Pose2d(startPose.getX() - 30, startPose.getY() + 11, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(carouselPose.getX() + 1, carouselPose.getY() + 24, Math.toRadians(0));
        Pose2d beforeStraight = new Pose2d(-47.04,-84.23, Math.toRadians(-270));
        Pose2d warehouseAfterStraight = new Pose2d(45, -84.23, Math.toRadians(0));
        Pose2d beforeWarehouseGap = new Pose2d(carouselPose.getX() + 106, carouselPose.getY() - 15, Math.toRadians(0));
        Pose2d warePose = new Pose2d(carouselPose.getX() + 106, carouselPose.getY() - 15, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        Trajectory traj2hub = drive.trajectoryBuilder(startPose, true)
                .splineToLinearHeading(sharedHubPose, sharedHubPose.getHeading())
                .build();
        Trajectory middlePoint = drive.trajectoryBuilder(sharedHubPose)
                .splineToLinearHeading(midPoint, midPoint.getHeading())
                .build();
        Trajectory traj2carousel = drive.trajectoryBuilder(midPoint, true)
                .splineToLinearHeading(carouselPose, carouselPose.getHeading())
                .build();
        Trajectory traj2BeforeStraight = drive.trajectoryBuilder(carouselPose)
                .splineToLinearHeading(beforeStraight, beforeStraight.getHeading())
                .build();
        Trajectory straightAway = drive.trajectoryBuilder(beforeStraight)
                .lineTo(new Vector2d(warehouseAfterStraight.getX(), warehouseAfterStraight.getY()))
                .build();

        Trajectory trajUnitPark = drive.trajectoryBuilder(carouselPose)
                .splineToLinearHeading(parkPose, parkPose.getHeading())
                .build();

        Trajectory trajWarePark = drive.trajectoryBuilder(carouselPose)
                .lineTo(new Vector2d(warePose.getX(), warePose.getY()))
                .build();

        TrajectorySequenceBuilder tsBuilder = drive.trajectorySequenceBuilder(startPose);
        drive.intake.clawClose();
        sleep(500);
        drive.intake.setGantryPosition(RobotIntake.ARM_LAYER3_POSZ, RobotIntake.MIN_HPOS);

        drive.intake.update();
        sleep(500);
        drive.followTrajectory(traj2hub);
        drive.intake.clawOpen();
        drive.intake.magnetRelease();
        sleep(500);
        drive.intake.setGantryPosition(RobotIntake.ARM_INTAKE_POS, RobotIntake.MAX_HPOS);
        drive.followTrajectory(middlePoint);
        drive.followTrajectory(traj2carousel);

        TrajectorySequence trajectories = tsBuilder.build();

        drive.followTrajectorySequence(trajectories);

        drive.intake.carousel.setPower(.35);
        driftDrive(DRIFT_XPOW, DRIFT_YPOW, 5);
        drive.intake.carousel.setPower(0);
        drive.update();
        Pose2d afterDuckPose = drive.getPoseEstimate();

        drive.setPoseEstimate(carouselPose);
        tsBuilder = drive.trajectorySequenceBuilder(carouselPose);

        if (doWarehousePark) {
            tsBuilder.addTrajectory(traj2BeforeStraight);
            tsBuilder.addTrajectory(straightAway)
                    .turn(Math.toRadians(-90));
        } else {
            tsBuilder.addTrajectory(trajUnitPark);
        }

        trajectories = tsBuilder.build();
        drive.followTrajectorySequence(trajectories);

    }

    public void red2() throws InterruptedException {
        Pose2d startPose = new Pose2d(12.4, -60.4, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        while (!opmode.isStopRequested()) {
            double pauseSeconds = .25;
            TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                    .forward(20)
                    .waitSeconds(pauseSeconds)
                    .turn(Math.toRadians(45))
                    .waitSeconds(pauseSeconds)
                    .forward(15)
                    .waitSeconds(pauseSeconds)
                    .back(15)
                    .waitSeconds(pauseSeconds)
                    .turn(Math.toRadians(-45))
                    .waitSeconds(pauseSeconds)
                    .back(20 - .5)
                    .waitSeconds(pauseSeconds)
                    .turn(Math.toRadians(-90))
                    .waitSeconds(pauseSeconds)
                    .forward(36)
                    .waitSeconds(pauseSeconds)
                    .turn(Math.toRadians(45))
                    .waitSeconds(pauseSeconds)
                    .turn(Math.toRadians(-45))
                    .waitSeconds(pauseSeconds)
                    .back(36)
                    .waitSeconds(pauseSeconds)
                    .turn(Math.toRadians(90))
                    .back(.5)
                    .build();
            drive.followTrajectorySequence(traj);
        }
    }

    public void blue1() throws InterruptedException {
        Pose2d startPose = new Pose2d(-36, 60.4, Math.toRadians(-90));
        Pose2d sharedHubPose = new Pose2d(-22.19, 35.65, Math.toRadians(315));
        Pose2d carouselPose = new Pose2d(-61.53, 50.76, Math.toRadians(45-90));
        Pose2d midPoint = new Pose2d(-40, 40, Math.toRadians(45-90));
        Pose2d parkPose = new Pose2d(-61.85, 35.96, Math.toRadians(0));
        Pose2d beforeStraight = new Pose2d(-47.04, 84.23, Math.toRadians(270));
        Pose2d warehouseAfterStraight = new Pose2d(45, 84.23, Math.toRadians(0));
        Pose2d beforeWarehouseGap = new Pose2d(10, 70, Math.toRadians(10));
        Pose2d warePose = new Pose2d(45, 70, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj2hub = drive.trajectoryBuilder(startPose, false)
                .splineToLinearHeading(sharedHubPose, sharedHubPose.getHeading())
                .build();
        Trajectory middlePoint = drive.trajectoryBuilder(sharedHubPose)
                .splineToLinearHeading(midPoint, midPoint.getHeading())
                .build();

        Trajectory traj2carousel = drive.trajectoryBuilder(midPoint, true)
                .splineToLinearHeading(carouselPose, carouselPose.getHeading())
                .build();

        Trajectory trajUnitPark = drive.trajectoryBuilder(carouselPose)
                .splineToLinearHeading(parkPose, parkPose.getHeading())
                .build();
        Trajectory traj2BeforeStraight = drive.trajectoryBuilder(parkPose)
                .splineToLinearHeading(beforeStraight, beforeStraight.getHeading())
                .build();
        Trajectory traj2Straight = drive.trajectoryBuilder(beforeStraight)
                .lineTo(new Vector2d(warehouseAfterStraight.getX(), warehouseAfterStraight.getY()))
                .build();

        TrajectorySequenceBuilder tsBuilder = drive.trajectorySequenceBuilder(startPose);

       // tsBuilder = tsBuilder
       //         .addTrajectory(traj2hub)
         //       .addTrajectory(traj2carousel);

        TrajectorySequence trajectories;
     // trajectories = tsBuilder.build();

    //    drive.followTrajectorySequence(trajectories);
        drive.intake.clawClose();
        sleep(500);
        drive.intake.setGantryPosition(RobotIntake.ARM_LAYER3_POSZ,RobotIntake.MIN_HPOS);
       // wait(2);
//        convertBarcode(vision.getBarcodeTSEUpdated(2));
     //   wait(2);
        drive.intake.update();
        sleep(500);
        drive.followTrajectory(traj2hub);
        drive.intake.clawOpen();
        drive.intake.magnetRelease();
        sleep(500);
        drive.intake.setGantryPosition(RobotIntake.ARM_INTAKE_POS, RobotIntake.MAX_HPOS);
        drive.followTrajectory(middlePoint);
        drive.followTrajectory(traj2carousel);

        drive.intake.carousel.setPower(-.35);
        driftDrive(DRIFT_XPOW, DRIFT_YPOW, 5);
        drive.intake.carousel.setPower(0);
        Pose2d afterDuckPose = drive.getPoseEstimate();

        drive.setPoseEstimate(carouselPose);
        tsBuilder = drive.trajectorySequenceBuilder(carouselPose);

        if (doWarehousePark) {
            tsBuilder.addTrajectory(traj2BeforeStraight);
            tsBuilder.addTrajectory(traj2Straight);
            drive.intake.setGantryPosition(RobotIntake.ARM_INTAKE_POS, RobotIntake.MAX_HPOS/2);
        } else {
            tsBuilder.addTrajectory(trajUnitPark);
        }

        trajectories = tsBuilder.build();
        drive.followTrajectorySequence(trajectories);
    }

    public void blue2() throws InterruptedException {
        Pose2d startPose = new Pose2d(12.4, -60.4, Math.toRadians(90));
    }

    public void driftDrive(double x, double y, double timeouts) {
        ElapsedTime timer = new ElapsedTime();
        while (!opmode.isStopRequested() && timer.seconds() < timeouts) {
            drive.setWeightedDrivePower(
                    new Pose2d(x, y, 0)
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
        drive.setWeightedDrivePower( new Pose2d(0, 0, 0));
        drive.update();
    }
    public void convertBarcode(int returnvalue) {
        if (returnvalue == 1) {
            drive.intake.setGantryPosition(RobotIntake.ARM_LAYER1_POSZ, RobotIntake.ARM_LAYER1_POSXY);
        } else if (returnvalue == 2) {
            drive.intake.setGantryPosition(RobotIntake.ARM_LAYER2_POSZ, RobotIntake.ARM_LAYER2_POSXY);
        } else if (returnvalue == 3) {
            drive.intake.setGantryPosition(RobotIntake.ARM_LAYER3_POSZ, RobotIntake.ARM_LAYER3_POSXY);
        } else {
            telemetry.addData("No objects found in vision", 404);
        }
    }
}
