package org.firstinspires.ftc.teamcode.xdrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name = "Auto", group = "7079")
public class Auto extends LinearOpMode {
    Robot robot = new Robot();
    DriveBrain driveBrain;
    double fixedHeading = 0;
    double verySlowPower = 0.1;
    double slowPower=0.2;
    double mediumPower=0.3;
    double halfPower = 0.5;
    double highPower = 0.7;
    double veryHighPower = 1;
    double shortTimeout=1.5;
    double mediumTimeout=4;
    double highTimeout = 7;
    int armMarkerPos = 0;
    private ElapsedTime runtime = new ElapsedTime();

    VisionBrain vision;

//    static final double     FORWARD_SPEED = 0.6;
//    static final double     TURN_SPEED    = 0.5;
//    static final double     STRAFE_SPEED  = 0.5;
    static final double     POWER         = 1.0;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AY7lK0j/////AAABmffl0hEQlUFfjdc9h8Aw+t5/CrgiSiIgNkZKZcw3qdOlnNEv3HarcW4e1pfYY5Nq+4XVrrnhKKNBeR/S08U41ogd0NpmWwOPgttli7io4p8WtbgWj+c/WL9uDzZK9u03K3Kfx+XFxdk/vy0tnFKCPg5w9M5iy7QQP2SDHFDJuhcAOtsayV8n8hQvB528RDRDykBtXei/V6xhN/qLc+S1Gp7eS0ZzpDFnT+uED0CwYK+oaWKNsPPv+3u9tCwofQ5PaRHlN05kH4V97Nn0N7WquSmDpcCZpAVqI1QnMEi7Fm9rvJgET+4OIlx4ZueF3ZTuXtJJSaEJ8Y6CEy9F7FS0RnlVtt4QlqpQVSmWmJQWYBNu";


    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;
    public void runOpMode() {
       
        waitForStart();



        robot.init(hardwareMap, telemetry);
        robot.setDriveStopModeBreak();
        driveBrain = new DriveBrain(robot, this);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        driveBrain.setZeroHeading();
//        autoPosTest();
        autoPos1(false, true);
        robot.setDriveStop();
    }
    //Side = true is blue
    //Side = false is red
    //carousel = false then we don't do carousel
    //carousel = true then we do carousel
    public void autoPos1(boolean sideBlue, boolean doCarousel) {
        int barcode = 0; //TODO get barcode number from vision
        int angleModifier = 1;
        if (sideBlue) angleModifier=-1;
        telemetry.addData("Starting!",0);
        //deliver preloaded box
        driveBrain.setArmMotorPosition(Robot.ARM_PARK_POS);//moves arm straight up
        driveBrain.maintTime(1.5);
        driveBrain.driveDistance(1,mediumPower, shortTimeout);//move forward 1in
        driveBrain.rotateToHeadingAbsolute(-30*angleModifier,2,0.5,mediumTimeout);
        driveBrain.driveDistance(30,halfPower,mediumTimeout);//drives to shipping hub
        telemetry.addData("Moved to shipping hub!",1);
        if (barcode==1) armMarkerPos = Robot.ARM_LAYER1_POS;
        if (barcode==2) armMarkerPos = Robot.ARM_LAYER2_POS;
        if (barcode==3) armMarkerPos = Robot.ARM_LAYER3_POS;
        robot.setArmMotorPosition(armMarkerPos);//TODO make it so that when the barcode is read then it goes to that level
        telemetry.addData("Placed Block!",2);
        //drives to warehouse
        if (doCarousel) {
            //gets duck off of carousel
            driveBrain.driveDistance(1,halfPower,shortTimeout);//drives 1 inch in order to get enough room to spin
            driveBrain.rotateToHeadingAbsolute(110*angleModifier,3,halfPower,mediumTimeout);//rotates so that it is facing carousel
            driveBrain.driveDistance(68, highPower, mediumTimeout);//drives to carousel
            driveBrain.carouselMoves();//moves the carousel wheel
            telemetry.addData("Moved the Carousel",3);
//          Gets a block from the warehouse and delivers it
            driveBrain.driveDistance(-1, mediumTimeout, shortTimeout);//goes back so that it has room to turn
            driveBrain.rotateToHeadingAbsolute(270*angleModifier,2,mediumPower,highTimeout);
            driveBrain.driveDistance(96,halfPower, highTimeout);//drives to warehouse
        }
        else {
            driveBrain.driveDistance(-1, mediumTimeout, shortTimeout);//goes back so that it has room to turn
            driveBrain.rotateToHeadingAbsolute(60*angleModifier,2,mediumPower,highTimeout);
            driveBrain.driveDistance(48,halfPower, highTimeout);//drives to warehouse
        }
        telemetry.addData("Drove to Warehouse!",4);
//        robot.pusherOpen();
//        driveBrain.setArmMotorPosition(128);
//        driveBrain.rotateToHeadingRelative(30, 1, mediumPower, shortTimeout);
//        driveBrain.setArmMotorPosition(353);
//        //TODO make it so that the arm and claw work together to get a block.
//        driveBrain.driveDistance(-72,halfPower,highTimeout);
//
//        driveBrain.rotateToHeadingAbsolute(360*angleModifier,2,halfPower,highTimeout);
//        driveBrain.driveDistance(33, highPower,highTimeout);//drives to shipping hub
//        telemetry.addData("Drove to Shipping Hub!",5);
//        robot.pusherClose();
//        telemetry.addData("Placed Block!",6);
//        //Parks in the warehouse
//        driveBrain.driveDistance(-33, slowPower, shortTimeout);//goes back so that it has enough room to turn
//        driveBrain.rotateToHeadingAbsolute(270*angleModifier,2,halfPower,mediumTimeout);
//        driveBrain.driveDistance(72, halfPower, mediumTimeout);//drives to go inside the warehouse
//        telemetry.addData("Drove to Warehouse!",7);
//        //end
//        telemetry.addData("Autonomous For Resource Depot Complete!",8);
    }
    public void autoPos2(boolean side) {
        int barcode = 0; //TODO get barcode number from vision
        int angleModifier = 1;
        if (side) angleModifier=-1;
        robot.setArmMotorPosition(750);
//        driveBrain.driveDistance();
    }
    public void autoPosTest(){
        driveBrain.driveDistance(20,mediumPower, 1);
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(90,3,mediumPower,3);
        sleep(5000);
        driveBrain.driveDistance(20,mediumPower,3);//drives to shipping hub
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(180,3,mediumPower,3);//rotates so that it is facing carousel
        sleep(5000);
        driveBrain.carouselMoves();//moves the carousel wheel
        sleep(5000);
        driveBrain.driveDistance(20, mediumPower,3);//drives to shipping hub
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(270,3,mediumPower,2);
        sleep(5000);
        driveBrain.driveDistance(20, mediumPower, 3);//drives to carousel
        sleep(5000);
        driveBrain.rotateToHeadingAbsolute(360,3,mediumPower,5);
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    private double getBarcodeValue(){
        double barcode = 1;

        return barcode;
    }
}
