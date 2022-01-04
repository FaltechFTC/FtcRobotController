package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility;

public class TeleBrain {
    Robot robot = new Robot();
    DriveBrain driveBrain;
    double fixedHeading = 0;
    double armOffset = 0.0;
    final double ARM_SPEED = 0.05;
    double armPos = 0;
    boolean lastTMode = false;

    ElapsedTime timer;
    int cycles;

    static final double DRIVE_OVERDRIVE_MULTIPLE = 2;

    Telemetry telemetry;

    public void init(OpMode opmode) {
        telemetry = opmode.telemetry;
        robot.init(opmode.hardwareMap, telemetry);
        driveBrain = new DriveBrain(robot, opmode);

        timer = new ElapsedTime();
        cycles = 0;

        telemetry.addData("Zero Offset", Robot.zeroHeadingOffset);

    }

    public void doDriving(double drive_forward, double drive_strafe, double drive_rotate,
                          boolean drive_heading_lock, boolean drive_tmode,
                          boolean drive_overdrive, boolean drive_rotate_90, boolean drive_global, boolean reset_heading) {
        if (reset_heading) {
            robot.setZeroHeading();
        }

        // DRIVING *******************************************
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

    public void doIntake(double arm_power, boolean pusher_cycle,
                         boolean arm_park, boolean arm_layer1, boolean outTakePos,
                         boolean intakePos, boolean wrist_up, boolean wrist_down) {

        if (wrist_up) robot.setWristOffset(robot.getWristOffset() + .01);
        if (wrist_down) robot.setWristOffset(robot.getWristOffset() - .01);
        robot.wristMove();

        // PUSHER **************************
        if (pusher_cycle && driveBrain.pusherTimer == null) {
            driveBrain.pusherStart(500);
        } else if (driveBrain.pusherTimer != null) driveBrain.pusherMaint();

        // ARM **************************************
        if (Robot.useArm) {
            if (arm_park) {
                armPos = Robot.ARM_PARK_POS;
            } else if (arm_layer1) {
                armPos = Robot.ARM_LAYER1_POS;
            }
//            else if (arm_intake) {
//                armPos = Robot.ARM_INTAKE_POS;
//            }
            else if (intakePos) {
                armPos = Robot.ARM_INTAKE_POS;
                robot.wristMove(.53);
            }//easter eggo
            else if (outTakePos) {
                armPos = Robot.ARM_LAYER1_POS;
                robot.wristMove(.03);
            } else
                armPos += arm_power;
            armPos = Utility.clipToRange(armPos, 1000, 0);
            robot.setArmMotorPosition(armPos);
        }
    }

    public void doCarousel(double carousel_power, boolean carousel_cycle_left,
                           boolean carousel_cycle_right) {

        if (Robot.useCarousel) {
            if (carousel_cycle_left && driveBrain.carouselTimer == null) {
                driveBrain.carouselStart(false);
            } else if (carousel_cycle_right && driveBrain.carouselTimer == null) {
                driveBrain.carouselStart(true);
            }

            if (driveBrain.carouselTimer != null) {
                driveBrain.carouselMaint();
            } else
                robot.carousel.setPower(carousel_power);
//            telemetry.addData("Carousel Power:", carousel_power);
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