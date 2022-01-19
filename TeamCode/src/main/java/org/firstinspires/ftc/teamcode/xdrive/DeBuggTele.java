package org.firstinspires.ftc.teamcode.xdrive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Utility;
@Config
@TeleOp(name = "DeBuggTele", group = "7079")
public class DeBuggTele extends OpMode {
    public static double xyMultiplier = -0.01;
    public static double zMultiplier = -12.0;
    boolean clawButtonPressed = false;
    TeleBrain brain = new TeleBrain();

    @Override
    public void init() {
        brain.init(this);
        brain.robot.intake.zEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brain.robot.intake.zEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        doDriving();
        doIntake();
        doCarousel();


        brain.robot.reportEncoders();
        brain.robot.intake.reportIntake();
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
        boolean drive_overdrive = gamepad1.right_bumper;
        boolean drive_rotate_90 = gamepad1.dpad_left;
        boolean drive_global = gamepad1.dpad_right;
        boolean reset_heading = gamepad1.dpad_down;

        brain.doDriving(drive_forward, drive_strafe, drive_rotate, drive_heading_lock,
                drive_tmode, drive_overdrive, drive_rotate_90, drive_global, reset_heading);
    }//not an easter egg

    public void doIntake() {
//        double pusher_pos = gamepad2.left_trigger;
        double zPower = zMultiplier * Utility.deadStick(gamepad2.left_stick_y);
        if (zPower > 0) zPower *= 2;
        double xyPower = xyMultiplier * Utility.deadStick(gamepad2.left_stick_x);
        boolean claw = gamepad2.right_bumper;
        boolean clawToggle = claw && !clawButtonPressed;
        clawButtonPressed = claw;
        boolean arm_park = gamepad1.y || gamepad2.y;
        boolean arm_layer1 = gamepad1.a || gamepad2.x;//in here magnus thinks that arm layer one means shared hub
        boolean outTakePos = gamepad2.a;
        boolean intakePos = gamepad2.b || gamepad1.b;
        boolean scoreTop = gamepad1.y;
        boolean topperPos = gamepad1.x;
        boolean magnet = gamepad2.left_bumper;

        brain.doIntake(zPower, xyPower, magnet, arm_park,
                arm_layer1, outTakePos, intakePos, clawToggle);
    }

    public void doCarousel() {
        double carousel_power = Utility.deadStick(gamepad2.right_stick_x);
        boolean carousel_cycle_left = gamepad2.dpad_left;
        boolean carousel_cycle_right = gamepad2.dpad_right;
        brain.doCarousel(carousel_power, carousel_cycle_left, carousel_cycle_right);
    }

//    public void saveLastPos() {
//        if (gamepad2.right_bumper && gamepad2.a) {
//            int x = 0;
//            while (x == 3) {
//                double currentwrist = robot.getWristOffset();
//                double currentArm = robot.getArmPosition();
//            }
//            double currentwrist = robot.getWristOffset();
//            double currentArm = robot.getArmPosition();
//
//        }
//
//    }

}
//Hi this is matthew, tell me if you see this!
//I saw it
//Me II