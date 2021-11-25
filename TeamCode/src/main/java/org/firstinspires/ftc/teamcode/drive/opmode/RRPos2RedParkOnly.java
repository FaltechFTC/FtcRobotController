package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
public class RRPos2RedParkOnly extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12.4, -60.4, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        while (!isStopRequested()) {
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
}
