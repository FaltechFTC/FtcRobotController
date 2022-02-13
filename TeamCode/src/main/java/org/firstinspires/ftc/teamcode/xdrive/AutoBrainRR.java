//mau is dumb lol
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


    public static double VUPC =0.006, VDPC=0.0045;
    public static double RED1BSX = -40.04;
    public static double RED1BSY = -84.23;
    public static int RED1BSH = -270;

    public static int iwilldothislater = 0;

    public static int RED1WASX = 45;
    public static double RED1WASY = -84.23;
    public static int RED1WASH = 0;

    public static int BLUE1MX = -42;
    public static int BLUE1MY = 45;

    public static int RED1CPX = -66;
    public static double RED1CPY = -49.4;
    public static int RED1CPH = 90;

    public static int RED1HUBX = -21;
    public static double RED1HUBY = -34.4;
    public static int RED1HUBH = 90 - 25;

    public static double BLUE1X = -15.19;
    public static double BLUE1Y = 26.65;

    public static double DRIFT_YPOW = .11;
    public static double DRIFT_XPOW = -.16;
    RobotRRDrive drive;
    VisionBrain vision;
    static public boolean useVision = true;
    Telemetry telemetry;
    LinearOpMode opmode;
    public boolean doWarehousePark = true;
    int barcode = 0;

    public void init(LinearOpMode opmode) {
        this.opmode = opmode;
        telemetry = opmode.telemetry;
        drive = new RobotRRDrive(opmode.hardwareMap, opmode.telemetry);

        RobotIntake.verticalDownPowerConstant = VDPC;
        RobotIntake.verticalUpPowerConstant = VUPC;

        if (useVision) {

            vision = new VisionBrain();
            vision.useWebCam = true;
            vision.showCamera = false; // useful for sighting on phone only
            vision.showCameraOD = true; // useful for seeing object detection on phone only
            vision.zoom = 1f;  // 1.0 is no zoom, greater number is greater zoom
            vision.init(opmode, telemetry);
            vision.activate();
            telemetry.addData("Status", "Vision Ready");
            telemetry.update();

        }
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (Exception e) {
            // eat it
        }
    }

    public void sleepWithUpdate(double milliseconds) {
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.milliseconds() < milliseconds) {
            drive.update();
            sleep(1);
            if (opmode.isStopRequested()) {
                // TODO: throw exception
            }
        }
    }

    public void red1warehouse() throws InterruptedException {
        Pose2d startPose = new Pose2d(-36, -60.4, Math.toRadians(90));
        Pose2d sharedHubPose = new Pose2d(RED1HUBX, RED1HUBY, Math.toRadians(RED1HUBH));
        Pose2d midPoint = new Pose2d(startPose.getX(), -43.4, Math.toRadians(90 - 25));
        Pose2d carouselPose = new Pose2d(RED1CPX, RED1CPY, Math.toRadians(RED1CPH));
        Pose2d parkPose = new Pose2d(carouselPose.getX(), -32.75+10, Math.toRadians(-90));
        Pose2d beforeStraight = new Pose2d(RED1BSX+5, RED1BSY, Math.toRadians(RED1BSH));
        Pose2d warehouseAfterStraight = new Pose2d(RED1WASX+15, RED1WASY, Math.toRadians(RED1WASH));
        Pose2d beforeWarehouseGap = new Pose2d(carouselPose.getX() + 106, carouselPose.getY() - 15, Math.toRadians(0));
        Pose2d bbStraight = new Pose2d(RED1BSX+10, carouselPose.getY(), -270);
        Pose2d warePose = new Pose2d(carouselPose.getX() + 106, carouselPose.getY() - 15, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        Trajectory traj2hub = drive.trajectoryBuilder(startPose, false)
                .splineToLinearHeading(sharedHubPose, sharedHubPose.getHeading())
                .build();
        Trajectory middlePoint = drive.trajectoryBuilder(sharedHubPose, true)
                .splineToLinearHeading(midPoint, midPoint.getHeading())
                .build();
        Trajectory traj2carousel = drive.trajectoryBuilder(midPoint, false)
                .splineToLinearHeading(carouselPose, carouselPose.getHeading())
                .build();
        Trajectory traj2BBStraight = drive.trajectoryBuilder(carouselPose)
                .lineTo(new Vector2d(RED1BSX+10, carouselPose.getY()))
                .build();
        Trajectory traj2BeforeStraight = drive.trajectoryBuilder(bbStraight)
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

        setupClaw();
        setupGantryToHubLevel();
        drive.followTrajectory(traj2hub);
        sleepWithUpdate(1000);
        releaseFreight();
        drive.intake.setXYPosition(RobotIntake.safeXY);
        sleepWithUpdate(250);

        drive.followTrajectory(middlePoint);
        drive.intake.setGantryPosition(RobotIntake.ARM_INTAKE_POSZ, RobotIntake.MAX_HPOS-.05);
        drive.followTrajectory(traj2carousel);

        drive.intake.carousel.setPower(.35);
        driftDrive(DRIFT_XPOW, DRIFT_YPOW, 5);
        drive.intake.carousel.setPower(0);
        drive.update();

        drive.setPoseEstimate(carouselPose);
        tsBuilder = drive.trajectorySequenceBuilder(carouselPose);

        if (doWarehousePark) {
            tsBuilder.addTrajectory(straightAway);
        } else {
            tsBuilder.addTrajectory(trajUnitPark);
        }
        TrajectorySequence trajectories = tsBuilder.build();
        drive.followTrajectorySequence(trajectories);
        setSafeUnitGantry(true);

    }

    public void red2warehouse() throws InterruptedException {
        Pose2d startPose = new Pose2d(12.4, -60.4, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj2hub = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .turn(Math.toRadians(60))
                .forward(20)
                .build();

        setupClaw();
        setupGantryToHubLevel();
        drive.followTrajectorySequence(traj2hub);
        sleepWithUpdate(1000);
        releaseFreight();
        drive.intake.setXYPosition(RobotIntake.safeXY);
        sleepWithUpdate(500);
        TrajectorySequence traj2warehouse = drive.trajectorySequenceBuilder(traj2hub.end())
                .back(20)
                .turn(Math.toRadians(-45))
                .back(30)
                .turn(Math.toRadians(-90))
                .forward(36)
                .turn(Math.toRadians(45))
                .forward(14)
                .turn(Math.toRadians(-45))
                .build();
        setSafeUnitGantry(true);
        drive.followTrajectorySequence(traj2warehouse);
    }

    public void blue1warehouse() throws InterruptedException {
        Pose2d startPose = new Pose2d(-36, 60.4, Math.toRadians(-90));
        Pose2d sharedHubPose = new Pose2d(BLUE1X-6.5, BLUE1Y+5, Math.toRadians(315));
        Pose2d carouselPose = new Pose2d(-61.53, 50.76, Math.toRadians(45));
        Pose2d midPoint = new Pose2d(BLUE1MX, BLUE1MY, Math.toRadians(45 - 90));
        Pose2d parkPose = new Pose2d(-67, 32.75-6.25, Math.toRadians(90));
        Pose2d beforeStraight = new Pose2d(-47.04+10, 84.23, Math.toRadians(0));
        Pose2d warehouseAfterStraight = new Pose2d(45+25, 84.23, Math.toRadians(-90));
        Pose2d mid2Point = new Pose2d(-47.04+10, 53, Math.toRadians(-90));
        Pose2d beforeWarehouseGap = new Pose2d(10, 70, Math.toRadians(10));
        Pose2d warePose = new Pose2d(45, 70, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj2hub = drive.trajectoryBuilder(startPose, false)
                .splineToLinearHeading(sharedHubPose, sharedHubPose.getHeading())
                .build();
        Trajectory middlePoint = drive.trajectoryBuilder(sharedHubPose, true)
                .splineToLinearHeading(midPoint, midPoint.getHeading())
                .build();

        Trajectory traj2carousel = drive.trajectoryBuilder(midPoint, false)
                .splineToLinearHeading(carouselPose, carouselPose.getHeading())
                .build();
        Trajectory trajMid2Point = drive.trajectoryBuilder(new Pose2d(-67.128, 48, -90))
                .lineTo(new Vector2d(mid2Point.getX(), mid2Point.getY()))
                .build();
        Trajectory trajUnitPark = drive.trajectoryBuilder(carouselPose)
                .splineToLinearHeading(parkPose, parkPose.getHeading())
                .build();
        Trajectory traj2BeforeStraight = drive.trajectoryBuilder(mid2Point)
                .splineToLinearHeading(beforeStraight, beforeStraight.getHeading())
                .build();
        Trajectory traj2Straight = drive.trajectoryBuilder(beforeStraight)
                .lineTo(new Vector2d(warehouseAfterStraight.getX(), warehouseAfterStraight.getY()))
                .build();

        TrajectorySequenceBuilder tsBuilder = drive.trajectorySequenceBuilder(startPose);
        TrajectorySequence trajectories;

        setupClaw();
        setupGantryToHubLevel();
        sleepWithUpdate(1000);
        drive.followTrajectory(traj2hub);

        releaseFreight();
        drive.intake.setXYPosition(RobotIntake.safeXY);
        sleepWithUpdate(250);

        drive.followTrajectory(middlePoint);
        drive.intake.setSafeGantryPosition();
        drive.followTrajectory(traj2carousel);

        drive.intake.carousel.setPower(-.35);
        driftDrive(0, DRIFT_YPOW+.05, 5);
        drive.intake.carousel.setPower(0);

        Pose2d afterDuckPose = drive.getPoseEstimate();
        drive.setPoseEstimate(carouselPose);
        tsBuilder = drive.trajectorySequenceBuilder(carouselPose);
        setSafeUnitGantry(false);
        if (doWarehousePark) {
            tsBuilder.addTrajectory(trajMid2Point);
            tsBuilder.addTrajectory(traj2BeforeStraight);
            tsBuilder.addTrajectory(traj2Straight);
            drive.intake.clawOpen();
        } else {
            tsBuilder.addTrajectory(trajUnitPark);
        }

        trajectories = tsBuilder.build();
        drive.followTrajectorySequence(trajectories);

        sleepWithUpdate(1000);
    }

    public void blue2warehouse() throws InterruptedException {
        Pose2d startPose = new Pose2d(12.4, 60.4, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj2hub = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .turn(Math.toRadians(-35))
                .forward(15)
                .build();
        TrajectorySequence traj2wall = drive.trajectorySequenceBuilder(traj2hub.end())
                .back(15)
                .turn(Math.toRadians(35))
                .back(30)
                .build();
        TrajectorySequence traj2warehouse = drive.trajectorySequenceBuilder(traj2wall.end())
                .turn(Math.toRadians(90))
                .forward(36)
                .turn(Math.toRadians(-45))
                .forward(25)
                .turn(Math.toRadians(45))
                .build();

        setupClaw();
        setupGantryToHubLevel();
        sleepWithUpdate(1000);
        drive.followTrajectorySequence(traj2hub);
        releaseFreight();
        drive.intake.setXYPosition(RobotIntake.safeXY);
        sleepWithUpdate(250);
        drive.followTrajectorySequence(traj2wall);
        setSafeUnitGantry(true);
        drive.followTrajectorySequence(traj2warehouse);

    }

    public void testAuto(){
        RobotIntake.verticalDownPowerConstant = VDPC;
        RobotIntake.verticalUpPowerConstant = VUPC;
        setupGantryToLevel(2);
        sleepWithUpdate(10000);
        drive.intake.setSafeGantryPosition();
        sleepWithUpdate(5000);

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
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        drive.update();
    }

    public void setupClaw() {
        drive.intake.clawClose();
        sleepWithUpdate(750);
    }

    public void setupGantryToHubLevel() {
        if(useVision)
            barcode=vision.getBarcodeTSEUpdated(1.5);
        telemetry.addData("Barcode:", barcode);
        telemetry.update();
        setupGantryToLevel(barcode);
        sleepWithUpdate(500);
    }


    public void setupGantryToLevel(int level) {
        telemetry.addData("Moving to level", level);
        if (level == 1) {
            drive.intake.setGantryPosition(RobotIntake.ARM_LAYER1_POSZ, RobotIntake.ARM_LAYER1_POSXY+.1);
        } else if (level == 2) {
            drive.intake.setGantryPosition(RobotIntake.ARM_LAYER2_POSZ+12, RobotIntake.ARM_LAYER2_POSXY);
        } else {
            drive.intake.setGantryPosition(RobotIntake.ARM_LAYER3_POSZ, RobotIntake.ARM_LAYER3_POSXY-.0274);
        }
    }

    public void setSafeUnitGantry(boolean clawOpen){
        if (clawOpen){drive.intake.clawOpen();}
        else{drive.intake.clawClose();}
        drive.intake.setGantryPosition(978, RobotIntake.MAX_HPOS);
    }

    public void releaseFreight() {
        drive.intake.clawMiddle();
        drive.intake.magnetRelease();
        sleepWithUpdate(500);
    }
    public void setBarcodeGantryPosition(int barcode){
        if (barcode ==1){
            setupGantryToLevel(1);
        }else if (barcode == 2){
            setupGantryToLevel(2);
        }else{
            setupGantryToLevel(3);
        }
    }

}
