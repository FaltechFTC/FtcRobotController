package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class XDriveTeleopVision extends OpMode {
    faltechBotXDrive robotXDrive = new faltechBotXDrive();
    DriveBrainXDrive driveBrain;
    XDriveVisionBrain visionXDrive;

    @Override
    public void init() {
        robotXDrive.init(hardwareMap);
        driveBrain = new DriveBrainXDrive(robotXDrive, this);
        visionXDrive = new XDriveVisionBrain();
    }

    @Override
    public void loop() {

        double forward  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

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


