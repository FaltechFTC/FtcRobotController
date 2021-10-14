package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "XDriveTeleop", group = "7079")
public class XDriveTeleop extends OpMode {
    faltechBotXDrive robotXDrive = new faltechBotXDrive();
    DriveBrainXDrive driveBrain;
    Utility Utility;
    double fixedHeading = 0;


    @Override
    public void init() {
        robotXDrive.init(hardwareMap, telemetry);
        driveBrain = new DriveBrainXDrive(robotXDrive, this);

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
            robotXDrive.setDriveStopModeBreak();
        }
        if (gamepad1.dpad_down) {
            robotXDrive.setDriveStopModeFloat();
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
            robotXDrive.setTwoWheelDrive(forward, strafe, rotate);
            return;
        }
        if (gamepad1.right_bumper) {
            forward = forward * 2;
            strafe = strafe * 2;
            rotate = rotate * 2;
        }
        double currentHeading = robotXDrive.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Our Heading", currentHeading);
        if (gamepad1.left_trigger == 0.00) {
            fixedHeading = currentHeading;
            if (gamepad1.a) {
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
            rotate = rotationCorrection + rotate;
            telemetry.addData("Fixed Heading", fixedHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Rotation Correction", rotationCorrection);

        }
        robotXDrive.setDrive(forward, strafe, rotate, 1);
        robotXDrive.reportEncoders();
        robotXDrive.calculateXDriveMotors(gamepad1.left_stick_x, gamepad1.left_stick_y, rotate);

                ;
        //robotXDrive.colorSensor();

        // if (gamepad1.a) {
        //   driveBrain.gyroDrive(.5, 10, 10);
        // driveBrain.gyroTurn(.5, 10);
        //
        // }
    }
}
