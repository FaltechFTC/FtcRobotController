package org.firstinspires.ftc.teamcode.xdrive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Utility;
@Config
public class TeleBrain {
    static final double DRIVE_OVERDRIVE_MULTIPLE = 2;
    final double ARM_SPEED = 0.05;
    RobotDrive robot = new RobotDrive();
    DriveBrain driveBrain;
    double fixedHeading = 0;
    double armOffset = 0.0;
    boolean lastTMode = false;
    ElapsedTime timer;
    int cycles;
    Telemetry telemetry;

    public void init(OpMode opmode) {
        telemetry = opmode.telemetry;
        robot.init(opmode.hardwareMap, telemetry);
        driveBrain = new DriveBrain(robot, opmode);

        timer = new ElapsedTime();
        cycles = 0;


        telemetry.addData("Zero Offset", RobotDrive.zeroHeadingOffset);

    }

    public void doDriving(double drive_forward, double drive_strafe, double drive_rotate,
                          boolean drive_heading_lock, boolean drive_tmode,
                          boolean drive_overdrive, boolean drive_rotate_90, boolean drive_global, boolean reset_heading) {
        if (reset_heading) {
            robot.setZeroHeading();
        }

        // DRIVING **************************************
        if (drive_overdrive) {
            drive_forward *= DRIVE_OVERDRIVE_MULTIPLE;
            drive_strafe *= DRIVE_OVERDRIVE_MULTIPLE;
            drive_rotate *= DRIVE_OVERDRIVE_MULTIPLE;
        }

        if (drive_tmode) {
            //calculate drive direction
            double direction = Math.atan2(drive_forward, drive_strafe);
            //offset by 45 degrees
            direction += Math.toRadians(-45);
            double power = Math.sqrt(drive_forward * drive_forward + drive_strafe * drive_strafe);
            /* calculate back to strafe and forward */
            drive_forward = Math.sin(direction) * power;
            drive_strafe = Math.cos(direction) * power;
        }

        double currentHeading = robot.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Raw Heading", robot.getRawHeading(AngleUnit.DEGREES));
        telemetry.addData("Heading", currentHeading);

        if (drive_global) {
            double direction = Math.atan2(drive_forward, drive_strafe);
            direction += Math.toRadians(-currentHeading);
            double power = Math.sqrt(drive_forward * drive_forward + drive_strafe * drive_strafe);
            drive_forward = Math.sin(direction) * power;
            drive_strafe = Math.cos(direction) * power;
        }

        if (!drive_heading_lock || drive_tmode != lastTMode) {
            fixedHeading = calculateLockHeading(currentHeading, 90, drive_tmode);
        }
        if (drive_heading_lock) {
            drive_rotate += calculateRotationCorrection(fixedHeading, currentHeading, .15, 45.0);
        }
        if (drive_rotate_90) {
            driveBrain.rotateToHeadingRelative(90, 3, 0.3, 3);
        }
        robot.setDrive(drive_forward, drive_strafe, drive_rotate, 1);

        lastTMode = drive_tmode; // remember
    }

    public void doIntake(double z_power, double xy_power, boolean magnet,
                         boolean downLevel, boolean upLevel, boolean arm_layer1,
                         boolean intakePos, boolean clawToggle) {
        // CLAW **************************************
        if (clawToggle) {
            robot.intake.clawToggle();
        }
        // Magnet **************************************
        if (magnet && robot.intake.magnetTimer == null) {
            robot.intake.clawOpen();
            robot.intake.magnetStart(500);
        }
        // ARM **************************************
        if (RobotIntake.useGantry) {

            double zPos = robot.intake.getZPosition();
            double xyPos = robot.intake.getXYPosition();

            if (arm_layer1) {
                zPos = robot.intake.ARM_LAYER1_POSZ;
            } else if (intakePos) {
                zPos = robot.intake.ARM_INTAKE_POSZ;
            } else if (upLevel) {
                if (zPos < robot.intake.ARM_INTAKE_POSZ) zPos = robot.intake.ARM_INTAKE_POSZ;
                else if (zPos < robot.intake.ARM_LAYER1_POSZ) zPos = robot.intake.ARM_LAYER1_POSZ;
                else if (zPos < robot.intake.ARM_LAYER3_POSZ) zPos = robot.intake.ARM_LAYER3_POSZ;
            } else if (downLevel) {
                if (zPos > robot.intake.ARM_LAYER3_POSZ) zPos = robot.intake.ARM_LAYER3_POSZ;
                else if (zPos > robot.intake.ARM_LAYER1_POSZ) zPos = robot.intake.ARM_LAYER1_POSZ;
                else if (zPos > robot.intake.ARM_INTAKE_POSZ) zPos = robot.intake.ARM_INTAKE_POSZ;
            } else {
                zPos += z_power;
            }

            robot.intake.setZPosition(zPos);

            if (xy_power!=0.0) {
                xyPos = robot.intake.getXYPosition();
                xyPos += xy_power;
                robot.intake.setXYPosition(xyPos);
            }
        }
        robot.intake.update();
    }

    public void doCarousel(double carousel_right, double carousel_left, boolean carousel_cycle_left,
                           boolean carousel_cycle_right) {

        if (RobotIntake.useCarousel) {
            if (carousel_cycle_left && robot.intake.carouselTimer == null) {
                robot.intake.carouselStart(false);
            } else if (carousel_cycle_right && robot.intake.carouselTimer == null) {
                robot.intake.carouselStart(true);
            }

            if (robot.intake.carouselTimer == null && Utility.deadStick(carousel_right) > 0.25) {
                robot.intake.setCarouselPower(carousel_right);
            }
            if (robot.intake.carouselTimer == null && Utility.deadStick(carousel_left) > .25) {
                robot.intake.setCarouselPower(-carousel_left);
            }
        }
    }

    double calculateLockHeading(double heading, double lockMultipleDeg, boolean tmode) {
        double lockOffset = tmode ? 0 : 45;
        double lockMultiple = Math.round((heading - lockOffset) / lockMultipleDeg);
        heading = lockMultiple * lockMultipleDeg + lockOffset;
        return Utility.wrapDegrees360(heading);
    }

    double calculateRotationCorrection(double fixedHeading, double currentHeading, double maxCorrectivePower, double maxError) {
        double headingError = Utility.wrapDegrees360(fixedHeading - currentHeading);
        headingError = Utility.clipToRange(headingError, maxError, -maxError);

        double rotationCorrection = maxCorrectivePower * headingError / maxError;
        /*
        if (rotationCorrection > 0 && rotationCorrection < 0.05) {

            rotationCorrection = 0.05;
        }
        if (rotationCorrection < 0 && rotationCorrection > -0.05) {
            rotationCorrection = -0.05;
        }
        */
        telemetry.addData("Fixed Heading", fixedHeading);
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("Rotation Correction", rotationCorrection);

        return rotationCorrection;
    }

    public void saveLastPos() {
    }

}
//Hi this is matthew, tell me if you see this!