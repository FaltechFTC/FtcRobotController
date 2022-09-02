package org.firstinspires.ftc.teamcode.xdrive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class XDriveRRTrackingLocalizer extends TwoTrackingWheelLocalizer {
    // pos X is forward
    // pos Y is strafe left
    // pos rotation is rotate left
//hi
    public static double ENC1_X = 3.89;
    public static double ENC1_Y = 3.89;
    public static double ENC1_DEG = -45;

    public static double ENC2_X = -3.89;
    public static double ENC2_Y = 3.89;
    public static double ENC2_DEG = 45;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder encoder1, encoder2;

    private RobotRRDrive drive;

    public XDriveRRTrackingLocalizer(HardwareMap hardwareMap, RobotRRDrive drive) {
        super(Arrays.asList(
                new Pose2d(ENC1_X, ENC1_Y, Math.toRadians(ENC1_DEG)),
                new Pose2d(ENC2_X, ENC2_Y, Math.toRadians(ENC2_DEG))
        ));

        this.drive = drive;

        encoder1 = new Encoder(hardwareMap.get(DcMotorEx.class, "fldrive"));
        encoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, "bldrive"));

        encoder1.setDirection(Encoder.Direction.REVERSE);
        encoder2.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        double correction = Math.sqrt(2);
        return WHEEL_RADIUS * 2.0 * Math.PI * GEAR_RATIO * ticks / correction / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(encoder1.getCurrentPosition()),
                encoderTicksToInches(encoder2.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(encoder1.getRawVelocity()),
                encoderTicksToInches(encoder2.getRawVelocity())
        );
    }
}