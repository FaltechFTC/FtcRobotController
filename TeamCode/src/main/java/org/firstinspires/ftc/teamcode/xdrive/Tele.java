package org.firstinspires.ftc.teamcode.xdrive;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility;

@TeleOp(name = "Tele", group = "7079")
public class Tele extends OpMode {
    Robot robot = new Robot();
    DriveBrain driveBrain;
    double fixedHeading = 0;
    double armOffset = 0.0;
    final double ARM_SPEED = 0.05;
    double armPos = 0;

    LynxModule.BulkCachingMode readMode;
    ElapsedTime timer;
    int cycles;

    static final double DRIVE_OVERDRIVE_MULTIPLE = 2;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        driveBrain = new DriveBrain(robot, this);

        readMode = LynxModule.BulkCachingMode.OFF;
        timer = new ElapsedTime();
        cycles = 0;
    }

    @Override
    public void loop() {
        double carousel_power = Utility.deadStick(gamepad2.right_stick_x);
        double pusher_pos = gamepad2.left_trigger;
        double arm_power = -2.0 * Utility.deadStick(gamepad2.left_stick_y);
        if (arm_power > 0) arm_power *= 2;
        double drive_forward = -0.5 * Utility.deadStick(gamepad1.left_stick_y);
        double drive_strafe = 0.5 * Utility.deadStick(gamepad1.left_stick_x);
        double drive_rotate = 0.25 * -Utility.deadStick(gamepad1.right_stick_x);

        boolean drive_heading_lock = gamepad1.left_trigger > 0.05;
        boolean drive_tmode = gamepad1.right_trigger > 0.05;
        boolean drive_overdrive = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean pusher_cycle = gamepad1.left_bumper || gamepad1.a || gamepad2.left_bumper;
        boolean arm_park = gamepad1.y || gamepad2.y;
        boolean arm_layer1 = gamepad1.x || gamepad2.x;
        boolean arm_intake = gamepad1.b || gamepad2.b;
        boolean carousel_cycle = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean drive_rotate_90 = gamepad1.dpad_left || gamepad2.dpad_left;


        if (pusher_cycle) {
            robot.pusherClose();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                ; // eat it
            }
            robot.pusherOpen();
        }
        pusher_pos = Utility.clipToRange(pusher_pos, 1, 0);
        robot.intakePusher.setPosition(pusher_pos);
        if (drive_rotate_90) {
            driveBrain.rotateToHeadingRelative(90, 3, 0.3, 3);
            return;
        }
        if (Robot.useArm) {
            if (arm_park) {
                armPos = robot.ARM_PARK_POS;
            }
            else if (arm_layer1) {
                armPos = robot.ARM_LAYER1_POS;
            }
            else if (arm_intake) {
                armPos = robot.ARM_INTAKE_POS;
            }
            else
                armPos += arm_power;
            armPos = Utility.clipToRange(armPos, 1000, 0);
            robot.setArmMotorPosition(armPos);
        }
        if (drive_overdrive) {
            drive_forward *= DRIVE_OVERDRIVE_MULTIPLE;
            drive_strafe *= DRIVE_OVERDRIVE_MULTIPLE;
            drive_rotate *= DRIVE_OVERDRIVE_MULTIPLE;
        }

        if (drive_tmode) {
            //calculate drive direction
            double direction = Math.atan2(drive_forward,drive_strafe);
            //offset by 45 degrees
            direction += Math.toRadians(45);
            //calculate back to strafe and forward
            drive_forward = Math.sin(direction);
            drive_strafe = Math.cos(direction);
        }


        double currentHeading = robot.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Our Heading", currentHeading);
        if (!drive_heading_lock) {
            fixedHeading = currentHeading;
        }
        else {
            double headingError = Utility.wrapDegrees360(fixedHeading - currentHeading);
            if (headingError > 45) {
                headingError = 45;
            }
            if (headingError < -45) {
                headingError = -45;
            }
            if (Math.abs(headingError) < .5) {
                headingError = 0;
            }

            double rotationCorrection = headingError * 0.25 / 45.00;
            if (rotationCorrection > 0 && rotationCorrection < 0.05) {
                rotationCorrection = 0.05;
            }
            if (rotationCorrection < 0 && rotationCorrection > -0.05) {
                rotationCorrection = -0.05;
            }

            drive_rotate += rotationCorrection;
            telemetry.addData("Fixed Heading", fixedHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Rotation Correction", rotationCorrection);

        }
        if (Robot.useCarousel) {
            if (carousel_cycle) {
                driveBrain.carouselMoves();
            }
            robot.carousel.setPower(carousel_power);
            telemetry.addData("Carousel Power:", carousel_power);
        }

        robot.setDrive(drive_forward, drive_strafe, drive_rotate, 1);
        robot.reportEncoders();
        //robot.reportColor();

        // robotXDrive.calculateDrivePowersFSR(gamepad1.left_stick_x, gamepad1.left_stick_y, rotate);
        cycles++;
        //telemetry.addData("cycle time (ms): ", timer.milliseconds() / cycles);
        telemetry.update();
    }
}
