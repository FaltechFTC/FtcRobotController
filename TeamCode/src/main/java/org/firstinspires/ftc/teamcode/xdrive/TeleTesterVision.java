package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class TeleTesterVision extends OpMode {
    Robot robot = new Robot();
    DriveBrain driveBrain;
    VisionBrain visionBrain;

    @Override
    public void init() {
        robot.init(hardwareMap,telemetry);

        driveBrain = new DriveBrain(robot, this);

        visionBrain = new VisionBrain();
        visionBrain.useWebcam=true;
        visionBrain.zoom=1.0;
        visionBrain.init(this, true);

        telemetry.update();
    }

    @Override
    public void loop() {

        double forward  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;
        robot.setDrive(forward,strafe,rotate,1);

        robot.reportColor();

        visionBrain.process();
    }
}


