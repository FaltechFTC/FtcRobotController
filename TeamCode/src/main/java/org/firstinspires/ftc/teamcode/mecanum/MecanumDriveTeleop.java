package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 */
@TeleOp(name = "Teleop Mecanum", group = "7079")

public class MecanumDriveTeleop extends OpMode {
    faltechBotMecanum robot = new faltechBotMecanum(); // use the class created to define a Pushbot's hardware

    DriveBrainMecanum driveBrain;


    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        robot.init(hardwareMap);
        driveBrain = new DriveBrainMecanum(robot, this);

    }

    @Override
    public void loop() {


        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double forward = 0.375*gamepad1.left_stick_y;
        double strafe = 0.375*-gamepad1.left_stick_x;
        double rotate = 0.375*-gamepad1.right_stick_x;

        /* TO DO:
        - Incorporate dead stick logic
        */

        robot.setDrive(forward, strafe, rotate, 1);
//       if (gamepad1.a) {
//           driveBrain.gyroDrive(.5, 10, 10);
//           driveBrain.gyroTurn(.5, 10);
//       }
    }
}