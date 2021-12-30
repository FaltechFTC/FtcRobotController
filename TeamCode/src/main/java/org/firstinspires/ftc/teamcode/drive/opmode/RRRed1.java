package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(group = "drive")
public class RRRed1 extends LinearOpMode {
    public static double DRIFT_YPOW = .11;
    public static double DRIFT_XPOW = -.16;
    SampleMecanumDrive drive;



    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -60.4, Math.toRadians(90));
        Pose2d sharedHubPose = new Pose2d(startPose.getX() + 15, startPose.getY() + 26, Math.toRadians(90 - 25));
        Pose2d carouselPose = new Pose2d(startPose.getX() - 30, startPose.getY() + 11, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(carouselPose.getX() + 1, carouselPose.getY() + 24, Math.toRadians(0));
        Pose2d warePose = new Pose2d(carouselPose.getX() + 106, carouselPose.getY() -15, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        waitForStart();

        Trajectory traj2carousel = drive.trajectoryBuilder(sharedHubPose,true)
                .splineToLinearHeading(carouselPose,carouselPose.getHeading())
            double pauseSeconds = .25;
            TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                    .splineToLinearHeading(sharedHubPose,sharedHubPose.getHeading())
                    .waitSeconds(1.5) //TODO remember to drop the block
        ;

                    .build();

        drive.followTrajectorySequence(traj);

        Pose2d afterDuckPose = drive.getPoseEstimate();
        //TODO remember to start spinner
        driftDrive(DRIFT_XPOW, DRIFT_YPOW, 3);
        //TODO remember to stop spinner

        TrajectorySequence trajUnitPark = drive.trajectorySequenceBuilder(carouselPose)
                .splineToLinearHeading(parkPose,parkPose.getHeading())
                //.waitSeconds(pauseSeconds)
                .build();

        TrajectorySequence trajWarePark = drive.trajectorySequenceBuilder(carouselPose)
                .lineTo(new Vector2d(warePose.getX(), warePose.getY()))
                //.waitSeconds(pauseSeconds)
                .turn(Math.toRadians(-90))
                .build();

            drive.followTrajectorySequence(trajWarePark);

    }
    public void driftDrive(double x, double y, double timeouts){
        ElapsedTime timer = new ElapsedTime();
        while (!isStopRequested() && timer.seconds() < timeouts) {
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
        drive.setMotorPowers(0,0,0,0);
    }
}
