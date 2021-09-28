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
    @Override
    public void init() {
        robotXDrive.init(hardwareMap);
        driveBrain = new DriveBrainXDrive(robotXDrive, this);
    }

    @Override
    public void loop() {

        telemetry.addData("Our Heading", robotXDrive.getHeading(AngleUnit.DEGREES));

        double forward  = 1/2*gamepad1.left_stick_y;
        double strafe = -1/2*gamepad1.left_stick_x;
        double rotate  = -1/2*gamepad1.right_stick_x;

        robotXDrive.setDrive(forward,strafe,rotate,1);
        NormalizedRGBA colors = robotXDrive.getRGBA();
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.update();
//       if (gamepad1.a) {
//           driveBrain.gyroDrive(.5, 10, 10);
//           driveBrain.gyroTurn(.5, 10);
//       }
    }
}
