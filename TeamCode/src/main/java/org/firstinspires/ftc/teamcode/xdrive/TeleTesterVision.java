package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleVision", group = "7079")
public class TeleTesterVision extends OpMode {
    RobotDrive robot;
    DriveBrain driveBrain;
    VisionBrain visionBrain;
    boolean useDrive = false;

    @Override
    public void init() {
        if (useDrive) {
            robot = new RobotDrive();
            robot.init(hardwareMap, telemetry);
            driveBrain = new DriveBrain(robot, this);
        }

        visionBrain = new VisionBrain();
        visionBrain.useWebCam = true;
        visionBrain.showCamera = false; // useful for sighting on phone only
        visionBrain.showCameraOD = true; // useful for seeing object detection on phone only
        visionBrain.zoom = 1f;  // 1.0 is no zoom, greater number is greater zoom
        visionBrain.init(this, telemetry);
        visionBrain.activate();
        //visionBrain.initTfod();
        // visionBrain.initVuforia();
    }

    @Override
    public void loop() {
        visionBrain.getBarcodeTSEUpdated(2000);
        //teleDrive();
    }

    private void teleDrive() {
        if (robot == null) return;

        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        robot.setDrive(forward, strafe, rotate, 1);
    }
}


