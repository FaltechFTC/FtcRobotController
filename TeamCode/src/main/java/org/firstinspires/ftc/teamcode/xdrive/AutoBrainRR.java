package org.firstinspires.ftc.teamcode.xdrive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
public class AutoBrainRR {
    public static double DRIFT_YPOW = .11;
    public static double DRIFT_XPOW = -.16;
    public static double TEST_CONFIG = 1.0;
    SampleMecanumDrive drive;
    VisionBrain vision;
    boolean useVision = false;
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

    public void init(LinearOpMode opmode) {
        this.opmode = opmode;
        telemetry = opmode.telemetry;
//        telemetry.addData("Status", "RunOpMode");
//        telemetry.update();


        drive = new SampleMecanumDrive(opmode.hardwareMap);
//        drive.init(opmode.hardwareMap, telemetry);
//        drive.setDriveStopModeBreak();
//        drive.maxUpPower=.3;// slower during auto
//        drive.magnetEngage();
//        //robot.wristMove(0);
////        telemetry.addData("Status", "Robot Initialized");
////        telemetry.update();
//
//        drive.maxUpPower=.3; // slower during auto
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
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
            // eat it
        }
    }

    public void red1() throws InterruptedException {
        Pose2d startPose = new Pose2d(-36, -60.4, Math.toRadians(90));
        Pose2d sharedHubPose = new Pose2d(startPose.getX() + 15, startPose.getY() + 26, Math.toRadians(90 - 25));
        Pose2d carouselPose = new Pose2d(startPose.getX() - 30, startPose.getY() + 11, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(carouselPose.getX() + 1, carouselPose.getY() + 24, Math.toRadians(0));
        Pose2d warePose = new Pose2d(carouselPose.getX() + 106, carouselPose.getY() - 15, Math.toRadians(0));

        double pauseSeconds = .25;

        drive.setPoseEstimate(startPose);

//        waitForStart();

        Trajectory traj2hub = drive.trajectoryBuilder(startPose, true)
                .splineToLinearHeading(sharedHubPose, sharedHubPose.getHeading())
                .build();
        Trajectory traj2carousel = drive.trajectoryBuilder(sharedHubPose, true)
                .splineToLinearHeading(carouselPose, carouselPose.getHeading())
                .build();

        Trajectory trajUnitPark = drive.trajectoryBuilder(carouselPose)
                .splineToLinearHeading(parkPose, parkPose.getHeading())
                .build();

        Trajectory trajWarePark = drive.trajectoryBuilder(carouselPose)
                .lineTo(new Vector2d(warePose.getX(), warePose.getY()))
                .build();

        TrajectorySequenceBuilder tsBuilder = drive.trajectorySequenceBuilder(startPose);

        tsBuilder = tsBuilder
                .addTrajectory(traj2hub)
                .addTrajectory(traj2carousel);

        TrajectorySequence trajectories = tsBuilder.build();

        drive.followTrajectorySequence(trajectories);

        //TODO remember to start spinner
        driftDrive(DRIFT_XPOW, DRIFT_YPOW, 3);
        //TODO remember to stop spinner
        drive.update();
        Pose2d afterDuckPose = drive.getPoseEstimate();

        drive.setPoseEstimate(carouselPose);
        tsBuilder = drive.trajectorySequenceBuilder(carouselPose);

        boolean doWarehousePark = true;
        if (doWarehousePark) {
            tsBuilder.addTrajectory(trajWarePark)
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

//        waitForStart();

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
        Pose2d startPose = new Pose2d(-36, 60.4, Math.toRadians(-90));//TODO fix the x,y
        Pose2d sharedHubPose = new Pose2d(-22.19, 35.65, Math.toRadians(315));//TODO
        Pose2d carouselPose = new Pose2d(-61.93, 38.89, Math.toRadians(290));
        Pose2d parkPose = new Pose2d(-61.85, 35.96, Math.toRadians(0));
        Pose2d beforeWarhoseGap = new Pose2d(16.96, 64.93, Math.toRadians(0));
        Pose2d warePose = new Pose2d(45.16, 64.93, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj2hub = drive.trajectoryBuilder(startPose, true)
                .splineToLinearHeading(sharedHubPose, sharedHubPose.getHeading())
                .build();
        Trajectory traj2carousel = drive.trajectoryBuilder(sharedHubPose, true)
                .splineToLinearHeading(carouselPose, carouselPose.getHeading())
                .build();

        Trajectory trajUnitPark = drive.trajectoryBuilder(carouselPose)
                .splineToLinearHeading(parkPose, parkPose.getHeading())
                .build();

        Trajectory trajWareGap = drive.trajectoryBuilder(carouselPose)
                .splineToLinearHeading(beforeWarhoseGap, beforeWarhoseGap.getHeading())
                .build();

        Trajectory trajWarePark = drive.trajectoryBuilder(beforeWarhoseGap)
                .lineTo(new Vector2d(warePose.getX(), warePose.getY()))
                .build();

        TrajectorySequenceBuilder tsBuilder = drive.trajectorySequenceBuilder(startPose);

        tsBuilder = tsBuilder
                .addTrajectory(traj2hub)
                .addTrajectory(traj2carousel);

        TrajectorySequence trajectories = tsBuilder.build();

        drive.followTrajectorySequence(trajectories);

        //TODO remember to start spinner
        driftDrive(DRIFT_XPOW, DRIFT_YPOW, 5);
        //TODO remember to stop spinner
        driftDrive(-DRIFT_XPOW, -DRIFT_YPOW, 3);
        drive.update();
        Pose2d afterDuckPose = drive.getPoseEstimate();

        drive.setPoseEstimate(carouselPose);
        tsBuilder = drive.trajectorySequenceBuilder(carouselPose);

        boolean doWarehousePark = true;
        if (doWarehousePark) {
            tsBuilder.addTrajectory(trajWareGap);
            tsBuilder.addTrajectory(trajWarePark)
                    .turn(Math.toRadians(-90));
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
        drive.setMotorPowers(0, 0, 0, 0);
    }
}
