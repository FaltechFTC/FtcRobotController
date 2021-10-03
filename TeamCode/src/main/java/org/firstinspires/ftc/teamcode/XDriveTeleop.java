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
    double fixedHeading = 0;



    @Override
    public void init() {
        robotXDrive.init(hardwareMap);
        driveBrain = new DriveBrainXDrive(robotXDrive, this);
    }

    @Override
    public void loop() {



        double forward = 0.5 * gamepad1.left_stick_y;
        double strafe = -0.5 * gamepad1.left_stick_x;
        double rotate = -0.5 * gamepad1.right_stick_x;
        if(gamepad1.right_bumper){
          forward = forward*2;
          strafe = strafe*2;
          rotate = rotate*2;
        }
        double currentHeading = robotXDrive.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Our Heading", currentHeading);
        if (gamepad1.left_trigger == 0.00){
            fixedHeading = currentHeading;
            if(gamepad1.right_trigger > 0.00){
                fixedHeading += 180;
            }
        }
        if (gamepad1.left_trigger > 0.00){

            double headingError = Utility.wrapDegrees360(fixedHeading-currentHeading);
            if (headingError>45){
                headingError = 45;
            }
            if(headingError < -45){
                headingError = -45;
            }
            double rotationCorrection = headingError*0.3/45.00;
            rotate = rotationCorrection + rotate;
            telemetry.addData("Fixed Heading", fixedHeading);
            telemetry.addData("Heading Error", headingError);
            telemetry.addData("Rotation Correction", rotationCorrection);

        }

        robotXDrive.setDrive(forward, strafe, rotate, 1);


       // if (gamepad1.a) {
         //   driveBrain.gyroDrive(.5, 10, 10);
           // driveBrain.gyroTurn(.5, 10);
        //
        // }
    }
}
