package org.firstinspires.ftc.teamcode.xdrive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Utility;
@Config
@TeleOp(name = "Tele", group = "7079")
public class Tele extends OpMode {
    public static double xyMultiplier = 0.01;
    public static double zMultiplier = -14.0;
    boolean clawButtonPressed = false;
    boolean globalButtonPressed = false;
    boolean isGlobalOn = false;
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
        boolean globalToggle = drive_global && !globalButtonPressed;
        globalButtonPressed = drive_global;
        if (globalToggle) {
            isGlobalOn = !isGlobalOn;
        }
        boolean reset_heading = gamepad1.dpad_down;

        brain.doDriving(drive_forward, drive_strafe, drive_rotate, drive_heading_lock,
                drive_tmode, drive_overdrive, drive_rotate_90, isGlobalOn, reset_heading);
    }//not an easter egg

    public void doIntake() {
//        double pusher_pos = gamepad2.left_trigger;
        double zPower = zMultiplier * Utility.deadStick(gamepad2.left_stick_y);
        if (zPower > 0) zPower *= 2;
        double xyPower = xyMultiplier * Utility.deadStick(gamepad2.right_stick_y);
        boolean claw = gamepad2.right_bumper;
        boolean clawToggle = claw && !clawButtonPressed;
        clawButtonPressed = claw;
//        boolean arm_park = gamepad1.y || gamepad2.y;
        boolean arm_layer1 = gamepad1.a || gamepad2.x;//in here magnus thinks that arm layer one means shared hub
//        boolean outTakePos = gamepad2.a;
        boolean intakePos = gamepad2.b || gamepad1.b;
//        boolean scoreTop = gamepad1.y;
//        boolean topperPos = gamepad1.x;
        boolean debug = gamepad2.back;

        boolean upLevel = gamepad1.y || gamepad2.dpad_up;
        boolean downLevel = gamepad1.x || gamepad2.dpad_down;
        boolean magnet = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean safeGantry = gamepad2.a;

        brain.doIntake(zPower, xyPower, safeGantry, magnet, downLevel, upLevel,
                arm_layer1, intakePos, clawToggle, debug);
    }

    public void doCarousel() {
        double carouselRight = Utility.deadStick(gamepad2.right_trigger);
        double carouselLeft = Utility.deadStick(gamepad2.left_trigger);
        boolean carousel_cycle_left = gamepad2.dpad_left;
        boolean carousel_cycle_right = gamepad2.dpad_right;
        brain.doCarousel(carouselRight, carouselLeft, carousel_cycle_left, carousel_cycle_right);
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
// ok cool hi dou lol me big bwain...