package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 *
 */
@TeleOp(name="TeleopTestMecanum", group="7079")
public class TeleopTest extends OpMode {
    faltechBot robot       = new faltechBot(); // use the class created to define a Pushbot's hardware

    DriveBrain driveBrain ;
    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        robot.init(hardwareMap);
        driveBrain = new DriveBrain(robot,this);

    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double forward  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;
        /* TO DO:
        - Incorporate dead stick logic
        */

       robot.setDrive(forward,strafe,rotate,1);
       NormalizedRGBA colors = robot.getRGBA();
       telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
       telemetry.update();
    }
}