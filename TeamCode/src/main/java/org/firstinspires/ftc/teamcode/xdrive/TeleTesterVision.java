package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleVisionTester Hub", group = "7079")
public class TeleTesterVision extends OpMode {
    Robot robot;
    DriveBrain driveBrain;
    VisionBrain visionBrain;
    boolean useDrive = false;
    boolean usingPhone=false;
    float zoom=1.2f;

    // Creating additional subclasses is a cheap way to create additional opmodes with slightly different configurations
    // that otherwise using the same core logic.
    @TeleOp(name = "TeleVisionTester Phone", group = "7079")
    public class TeleTesterVisionPhone extends TeleTesterVision {
        public TeleTesterVisionPhone() {
            super();
            usingPhone=true;  // overrides the parent class defaults
            zoom=1.0f;
        }
    }


    @Override
    public void init() {
        if (useDrive) {
            robot =new Robot();
            robot.init(hardwareMap, telemetry);
            driveBrain = new DriveBrain(robot, this);
        }

        visionBrain = new VisionBrain();
        visionBrain.useWebCam=!usingPhone; // if false, then assumes back of phone.
        visionBrain.showCamera=false; // useful for sighting on phone only
        visionBrain.showCameraOD=usingPhone; // useful for seeing object detection on phone only
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


