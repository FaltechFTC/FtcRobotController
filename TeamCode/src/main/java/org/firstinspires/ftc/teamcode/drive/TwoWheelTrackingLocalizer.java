package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;

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
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    // pos X is forward
    // pos Y is strafe left
    // pos rotation is rotate left

    public static double ENC1_X = 4;
    public static double ENC1_Y = 4;
    public static double ENC1_DEG = -45;

    public static double ENC2_X = -4;
    public static double ENC2_Y = 4;
    public static double ENC2_DEG =45;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder encoder1, encoder2;

    private SampleMecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
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
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
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
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(encoder1.getRawVelocity()),
                encoderTicksToInches(encoder2.getRawVelocity())
        );
    }
}