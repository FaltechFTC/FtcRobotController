package org.firstinspires.ftc.teamcode.xdrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utility;

@TeleOp(name = "Tele", group = "7079")
public class Tele extends OpMode {
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
        //brain.robot.reportDistance();
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
        boolean drive_global = gamepad1.a;
        boolean reset_heading = gamepad1.dpad_down;

        brain.doDriving(drive_forward, drive_strafe, drive_rotate, drive_heading_lock,
                drive_tmode, drive_overdrive,drive_rotate_90,drive_global, reset_heading);
    }

    public void doIntake() {
        double pusher_pos = gamepad2.left_trigger;
        double arm_power = -4.0 * Utility.deadStick(gamepad2.left_stick_y);
        if (arm_power > 0) arm_power *= 2;
        boolean pusher_cycle = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean arm_park = gamepad1.y || gamepad2.y;
        boolean arm_layer1 = gamepad1.x || gamepad2.x;
        boolean outTakePos = gamepad2.a;
        boolean intakePos = gamepad2.b || gamepad1.b;

        boolean wrist_up = gamepad2.dpad_up;
        boolean wrist_down = gamepad2.dpad_down;
        brain.doIntake(pusher_pos, arm_power, pusher_cycle, arm_park,
                arm_layer1, outTakePos, intakePos, wrist_up, wrist_down);
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