package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Utility;

@TeleOp(name = "TeleJ", group = "7079")
public class TeleJ extends OpMode {
    TeleBrain brain = new TeleBrain();

    @Override
    public void init() {
        brain.init(this);
    }

    @Override
    public void loop() {

        doDriving();
        doIntake();
        doCarousel();

        // OTHER *********************************************
        brain.robot.reportEncoders();
        //robot.reportColor();
        //telemetry.addData("cycle time (ms): ", timer.milliseconds() / cycles);
//        telemetry.addData("Distance Sensor Reading:", brain.robot.distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    public void doDriving() {
        double drive_forward = -0.5 * Utility.deadStick(gamepad1.left_stick_y);
        double drive_strafe = 0.5 * Utility.deadStick(gamepad1.left_stick_x);
        double drive_rotate = 0.25 * -Utility.deadStick(gamepad1.right_stick_x);

        boolean drive_heading_lock = gamepad1.left_trigger > 0.05;
        boolean drive_tmode = gamepad1.right_trigger > 0.05;
        boolean drive_overdrive = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean drive_rotate_90 = gamepad1.dpad_left;
        boolean drive_global = gamepad1.left_bumper;
        boolean reset_heading = gamepad1.dpad_down;

        brain.doDriving(drive_forward, drive_strafe, drive_rotate, drive_heading_lock,
                drive_tmode, drive_overdrive, drive_rotate_90, drive_global, reset_heading);
    }

    public void doIntake() {
        double zPower = -4.0 * Utility.deadStick(gamepad2.left_stick_y);
        if (zPower > 0) zPower *= 2;
        double xypower = -4.0 * Utility.deadStick(gamepad2.left_stick_y);
        if (xypower > 0) xypower *= 2;
        boolean pusher_cycle = gamepad1.a || gamepad2.left_bumper;
        boolean arm_park = gamepad1.y || gamepad2.y;
        boolean arm_layer1 = gamepad1.x || gamepad2.x;
        boolean outTakePos = gamepad2.a;
        boolean intakePos = gamepad2.b || gamepad1.b;
        boolean magnet_engage = gamepad2.dpad_up;
        brain.doIntake(zPower, xypower, pusher_cycle, arm_park,
                arm_layer1, outTakePos, intakePos, magnet_engage);
    }

    public void doCarousel() {
        double carousel_power = Utility.deadStick(gamepad2.right_stick_x);
        boolean carousel_cycle_left = gamepad2.dpad_left;
        boolean carousel_cycle_right = gamepad2.dpad_right;
        brain.doCarousel(carousel_power, carousel_cycle_left, carousel_cycle_right);
    }
}
//Hi this is matthew, tell me if you see this!
//I saw it