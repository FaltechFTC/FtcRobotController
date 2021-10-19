package org.firstinspires.ftc.teamcode.xdrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Tele", group = "7079")
public class Tele extends OpMode {
    Robot robot = new Robot();
    DriveBrain driveBrain;
    org.firstinspires.ftc.teamcode.Utility Utility;
    double fixedHeading = 0;
    double armOffset = 0.0;
    final double ARM_SPEED = 0.05;


    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        driveBrain = new DriveBrain(robot, this);

    }

    @Override
    public void loop() {
        boolean T_Mode = false;
        Utility.deadStick(gamepad1.left_stick_x);
        Utility.deadStick(gamepad1.left_stick_y);
        Utility.deadStick(gamepad1.right_stick_x);
        double forward = 0.5 * gamepad1.left_stick_y;
        double strafe = -0.5 * gamepad1.left_stick_x;
        double rotate = -0.25 * gamepad1.right_stick_x;
        if (gamepad1.dpad_up) {
            robot.setDriveStopModeBreak();
        }
        if (gamepad1.dpad_down) {
            robot.setDriveStopModeFloat();
        }
        if (gamepad1.b) {
            driveBrain.rotateToHeadingAbsolute(90, 3, 0.3, 4);

            return;
        }
        if (gamepad1.x) {
            driveBrain.rotateToHeadingRelative(90, 3, 0.3, 3);
            return;
        }
        if (gamepad1.right_trigger == 0.00) {
            telemetry.addData("T-Mode", T_Mode);
        }
        if (gamepad1.right_trigger > 0.00) {

            T_Mode = true;
            telemetry.addData("T-Mode", T_Mode);
            if (Math.abs(strafe) > Math.abs(forward)) {
                forward = 0;
                // because he are strafing we can kill the forward
                //  strafe = strafe;
            } else {
                strafe = 0;
                // because we are going forward, we will kill the strafe
                // forward = forward;
            }
            robot.setTwoWheelDrive(forward, strafe, rotate);
            return;
        }
        if (gamepad1.right_bumper) {
            forward = forward * 2;
            strafe = strafe * 2;
            rotate = rotate * 2;
        }
        double currentHeading = robot.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Our Heading", currentHeading);
        if (gamepad1.left_trigger == 0.00) {
            fixedHeading = currentHeading;
            if (gamepad1.a && gamepad1.left_trigger == 0.00) {
                fixedHeading += 180;
            }
        }
        if (gamepad1.left_trigger > 0.00) {

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
            // i am trash at coding
            rotate = rotationCorrection + rotate;
            telemetry.addData("Fixed Heading", fixedHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Rotation Correction", rotationCorrection);

        }
        if (robot.useArm) {
            if (gamepad2.dpad_up) robot.arm.setPower(.1);
            else if (gamepad2.dpad_down) robot.arm.setPower(-.1);
            else robot.arm.setPower(0);
//            if (gamepad2.dpad_right) {
//                armOffset += ARM_SPEED;
//                if (armOffset > 1) {armOffset =1;}
////                robot.arm.setPosition(armOffset);
//            }
//            if (gamepad2.dpad_left) {
//                armOffset -= ARM_SPEED;
//                if (armOffset < 0.0) {armOffset =0.0;}
//                robot.arm.setPower(.2);
////                robot.arm.setPosition(armOffset);
//            }
            telemetry.addData("Arm angle:", armOffset);
        }
        if (robot.useCarousel) {
            if (gamepad2.dpad_left) robot.carousel.setPower(.5);
            else if (gamepad2.dpad_right) robot.carousel.setPower(-.5);
            else robot.carousel.setPower(0);
        }
        robot.setDrive(forward, strafe, rotate, 1);
        robot.reportEncoders();

        // robotXDrive.calculateDrivePowersFSR(gamepad1.left_stick_x, gamepad1.left_stick_y, rotate);
    }
}
