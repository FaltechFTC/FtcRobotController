package org.firstinspires.ftc.teamcode.xdrive;
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
    boolean lastTMode = false;

    ElapsedTime timer;
    int cycles;

    static final double DRIVE_OVERDRIVE_MULTIPLE = 2;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        driveBrain = new DriveBrain(robot, this);

        timer = new ElapsedTime();
        cycles = 0;
    }

    @Override
    public void loop() {
        doDriving();
        doIntake();
        doCarousel();

        // OTHER *********************************************
        robot.reportEncoders();
        //robot.reportColor();

        cycles++;
        //telemetry.addData("cycle time (ms): ", timer.milliseconds() / cycles);
        telemetry.update();
    }
    public void doDriving() {
        double drive_forward = -0.5 * Utility.deadStick(gamepad1.left_stick_y);
        double drive_strafe = 0.5 * Utility.deadStick(gamepad1.left_stick_x);
        double drive_rotate = 0.25 * -Utility.deadStick(gamepad1.right_stick_x);

        boolean drive_heading_lock = gamepad1.left_trigger > 0.05;
        boolean drive_tmode = gamepad1.right_trigger > 0.05;
        boolean drive_overdrive = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean drive_rotate_90 = gamepad1.dpad_left;

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
            direction += Math.toRadians(45);
            double power = Math.sqrt(drive_forward*drive_forward+drive_strafe*drive_strafe);
            //calculate back to strafe and forward
            drive_forward = Math.sin(direction)*power;
            drive_strafe = Math.cos(direction)*power;
        }

        double currentHeading = robot.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Heading", currentHeading);
        if (!drive_heading_lock || drive_tmode!=lastTMode) {
            fixedHeading = calculateLockHeading(currentHeading, 90, drive_tmode);
        }
        if (drive_heading_lock) {
            drive_rotate += calculateRotationCorrection(fixedHeading, currentHeading, .25, 45.0);
        }
        if (drive_rotate_90) {
            driveBrain.rotateToHeadingRelative(90, 3, 0.3, 3);
        }
        robot.setDrive(drive_forward, drive_strafe, drive_rotate, 1);

        lastTMode=drive_tmode; // remember
    }

    public void doIntake() {
        double pusher_pos = gamepad2.left_trigger;
        double arm_power = -4.0 * Utility.deadStick(gamepad2.left_stick_y);
        if (arm_power > 0) arm_power *= 2;
        boolean pusher_cycle = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean arm_park = gamepad1.y || gamepad2.y;
        boolean arm_layer1 = gamepad1.x || gamepad2.x;
//        boolean arm_intake = gamepad1.b || gamepad2.b;
        boolean intakePos = gamepad2.b || gamepad1.b;

        if (gamepad2.dpad_up) robot.setWristOffset(robot.getWristOffset()+.01);
        if (gamepad2.dpad_down) robot.setWristOffset(robot.getWristOffset()-.01);
        robot.wristMove();

        // PUSHER **************************
        if (pusher_cycle && driveBrain.pusherTimer==null) {
            driveBrain.pusherStart();
        }
        else if(driveBrain.pusherTimer!=null) driveBrain.pusherMaint();
        else {
            pusher_pos = Utility.clipToRange(pusher_pos, 1, 0);
            robot.intakePusher.setPosition(pusher_pos);
        }

        // ARM **************************************
        if (Robot.useArm) {
            if (arm_park) {
                armPos = Robot.ARM_PARK_POS;
            }
            else if (arm_layer1) {
                armPos = Robot.ARM_LAYER1_POS;
            }
//            else if (arm_intake) {
//                armPos = Robot.ARM_INTAKE_POS;
//            }
            else if (intakePos) {
                armPos = Robot.ARM_INTAKE_POS;
                robot.setWristOffset(.53);
            }
            else
                armPos += arm_power;
            armPos = Utility.clipToRange(armPos, 1000, 0);
            robot.setArmMotorPosition(armPos);
        }
    }

    public void doCarousel() {
        double carousel_power = Utility.deadStick(gamepad2.right_stick_x);
        boolean carousel_cycle_left = gamepad2.dpad_left;
        boolean carousel_cycle_right = gamepad2.dpad_right;

        if (Robot.useCarousel) {
            if (carousel_cycle_left && driveBrain.carouselTimer==null) {
                driveBrain.carouselStart(false);
            }
            else if (carousel_cycle_right && driveBrain.carouselTimer==null) {
                driveBrain.carouselStart(true);
            }

            if (driveBrain.carouselTimer!=null) {
                driveBrain.carouselMaint();
            }
            else
                robot.carousel.setPower(carousel_power);
//            telemetry.addData("Carousel Power:", carousel_power);
        }
    }

    double calculateLockHeading(double heading, double lockMultipleDeg, boolean tmode){
        double lockOffset=tmode?0:45;
        double lockMultiple=Math.round((heading-lockOffset)/lockMultipleDeg);
        heading = lockMultiple*lockMultipleDeg+lockOffset;
        return Utility.wrapDegrees360(heading);
    }

    double calculateRotationCorrection(double fixedHeading, double currentHeading, double maxCorrectivePower, double maxError) {
        double headingError = Utility.wrapDegrees360(fixedHeading - currentHeading);
        headingError = Utility.clipToRange(headingError,maxError,-maxError);

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

}
