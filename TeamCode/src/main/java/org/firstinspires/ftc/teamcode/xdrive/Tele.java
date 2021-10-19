package org.firstinspires.ftc.teamcode.xdrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility;

@TeleOp(name = "Tele", group = "7079")
public class Tele extends OpMode {
    Robot robot = new Robot();
    DriveBrain driveBrain;
    private final ElapsedTime runtime = new ElapsedTime();

    double fixedHeading = 0;
    double currentHeading = 0;
    double lastLeftTrigger = 0;
    int loopsPerSecond = 0;
    long loops = 0;
    boolean tMode=false;
    static String fFormat ="%3.2f";
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        driveBrain = new DriveBrain(robot, this);
        currentHeading = robot.getHeading(AngleUnit.DEGREES);
    }

    @Override
    public void loop() {
        loops++;
        telemetry.addData("Loops per Second", loops / runtime.seconds());

        currentHeading = robot.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Heading", currentHeading);

        telemetry.addLine()
                .addData("Raw F",fFormat, gamepad1.left_stick_y)
                .addData("S",fFormat,  gamepad1.left_stick_x)
                .addData("R",fFormat,  gamepad1.right_stick_x);

        double forward =  -Utility.deadStick(gamepad1.left_stick_y);
        double strafe = Utility.deadStick(gamepad1.left_stick_x);;
        double rotate = Utility.deadStick(gamepad1.right_stick_x);

        if (gamepad1.right_bumper) { // turbo mode
            rotate *=.5;
        } else {                    // normal mode
            forward *= 0.4;
            strafe *= 0.4;
            rotate *= 0.25;
        }

        telemetry.addLine()
                .addData("Actual F",fFormat, forward)
                .addData("S",fFormat,  strafe)
                .addData("R",fFormat,  rotate);

        if (gamepad1.dpad_up)  robot.setDriveStopModeBreak();
        else if (gamepad1.dpad_down) robot.setDriveStopModeFloat();
        else if (gamepad1.a) driveBrain.rotateToHeadingAbsolute(0, 1, .35, 4);
        else if (gamepad1.b) driveBrain.rotateToHeadingAbsolute(180, 1, 0.35, 4);
        else if (gamepad1.x) driveBrain.rotateToHeadingRelative(-90, 2, 0.35, 4);
        else if (gamepad1.y) driveBrain.rotateToHeadingRelative(90, 2, 0.35, 4);
        else if (gamepad2.a) driveBrain.driveDistance(10,.3,4);
        else if (gamepad2.b) driveBrain.driveDistance(-10,.3,4);

        double leftTrigger = Utility.deadStick(gamepad1.left_trigger);
        if (leftTrigger > 0.00) {
            if (lastLeftTrigger == 0.00) fixedHeading = currentHeading;
            driveFixedHeading(forward, strafe, rotate);
        } else if (gamepad1.right_trigger > 0.00)
            drive2Wheel(forward,strafe,rotate);
        else
            robot.setDrive(forward, strafe, rotate, 1);

        telemetry.addData("T-Mode", tMode);

        lastLeftTrigger = leftTrigger;
    }

    public void drive2Wheel(double forward, double strafe, double rotate) {
        tMode = true;
        if (Math.abs(strafe) > Math.abs(forward))
            forward = 0;
        else
            strafe = 0;
        robot.setTwoWheelDrive(forward, strafe, rotate);
    }

    public void driveFixedHeading(double forward, double strafe, double rotate) {
        tMode = false;

        final double minRotPower=0.03, maxRotPower = 0.25;
        final double tolerance=0.5;
        final double maxErrorAngle = 45.0;

        double headingError = Utility.wrapDegrees360(currentHeading- fixedHeading);
        headingError = Utility.clipToRange(headingError, maxErrorAngle,-maxErrorAngle);
        if (Math.abs(headingError) < tolerance)
            headingError = 0;

        double rotationCorrection = headingError / maxErrorAngle * maxRotPower;
        if (rotationCorrection > 0 && rotationCorrection < minRotPower)
            rotationCorrection = minRotPower;
        else if (rotationCorrection < 0 && rotationCorrection > -minRotPower)
            rotationCorrection = -minRotPower;

        rotate = rotationCorrection + rotate;
        telemetry.addData("Fixed Heading", fixedHeading);
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("Rotation Correction", rotationCorrection);

        robot.setDrive(forward, strafe, rotate, 1);
    }
}
