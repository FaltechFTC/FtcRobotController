package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
@TeleOp(name = "TeleVision", group = "7079")
public class TeleTesterVision extends OpMode {
    Robot robotXDrive = new Robot();
    DriveBrain driveBrain;
    VisionBrain visionXDrive;
    @Override
    public void init() {
        robotXDrive.init(hardwareMap,telemetry);
        driveBrain = new DriveBrain(robotXDrive, this);
        visionXDrive = new VisionBrain();
        visionXDrive.init(this);
    }

    @Override
    public void loop() {

        double forward  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;
        if(gamepad1.a){
            visionXDrive.process();
        }
        robotXDrive.setDrive(forward,strafe,rotate,1);


//       if (gamepad1.a) {
//           driveBrain.gyroDrive(.5, 10, 10);
//           driveBrain.gyroTurn(.5, 10);
//       }
    }
}


