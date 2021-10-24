package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleVision", group = "7079")
public class TeleTesterVision extends OpMode {
    Robot robot;
    DriveBrain driveBrain;
    VisionBrain visionBrain;
    boolean useDrive = false;

    @Override
    public void init() {
        if (useDrive) {
            robot =new Robot();
            robot.init(hardwareMap, telemetry);
            driveBrain = new DriveBrain(robot, this);
        }

        visionBrain = new VisionBrain();
        visionBrain.showCamera=true; // useful for sighting on phone only
        visionBrain.showCameraOD=false; // useful for seeing object detection on phone only
        visionBrain.zoom=1.2f;  // 1.0 is no zoom, greater number is greater zoom
        visionBrain.init(this);
        visionBrain.activate();
    }

    @Override
    public void loop() {

//        if(gamepad1.a)
            visionBrain.process(2000);
  //      else
            teleDrive();
    }

    private void teleDrive() {
        if (robot == null ) return;

        double forward  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;
        robot.setDrive(forward,strafe,rotate,1);
    }
}


