/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.xdrive;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.xdrive.Robot;

import java.util.List;

import com.acmerobotics.dashboard.FtcDashboard;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 **/
public class VisionBrain {
    RobotDrive robot = new RobotDrive();
    OpMode opmode;
    private Telemetry telemetry = null;
    boolean useWebCam = false;
    boolean showCamera = false;
    boolean showCameraOD = false;
    float zoom = 0.8f;
    double returnvalue = 0;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod = null;

    final String VUFORIA_KEY = "AY7lK0j/////AAABmffl0hEQlUFfjdc9h8Aw+t5/CrgiSiIgNkZKZcw3qdOlnNEv3HarcW4e1pfYY5Nq+4XVrrnhKKNBeR/S08U41ogd0NpmWwOPgttli7io4p8WtbgWj+c/WL9uDzZK9u03K3Kfx+XFxdk/vy0tnFKCPg5w9M5iy7QQP2SDHFDJuhcAOtsayV8n8hQvB528RDRDykBtXei/V6xhN/qLc+S1Gp7eS0ZzpDFnT+uED0CwYK+oaWKNsPPv+3u9tCwofQ5PaRHlN05kH4V97Nn0N7WquSmDpcCZpAVqI1QnMEi7Fm9rvJgET+4OIlx4ZueF3ZTuXtJJSaEJ8Y6CEy9F7FS0RnlVtt4QlqpQVSmWmJQWYBNu";

    final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
/*
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Gold Mineral";
    private static final String LABEL_SECOND_ELEMENT = "Silver Mineral";
*/

    public void init(OpMode theopmode, Telemetry t) {
        telemetry = t;
        opmode = theopmode;

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
    }

    public void activate() {
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(zoom, 16.0 / 9.0);
        }

        opmode.telemetry.addData("Status", "Vision Activated");
        opmode.telemetry.update();

    }

    public void deactivate() {
        if (tfod != null) tfod.deactivate();
        opmode.telemetry.addData("Status", "Vision De-Activated");
        opmode.telemetry.update();
    }

    public void shutdown() {
        if (tfod != null) tfod.shutdown();
        opmode.telemetry.addData("Status", "Vision Shutdown");
        opmode.telemetry.update();
    }

    public void process(double timeout) {
        opmode.telemetry.addData("Status", "Processing!");

        if (tfod != null) {
            ElapsedTime timer = new ElapsedTime();

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = updatedRecognitions = tfod.getUpdatedRecognitions();
            ;
            while (updatedRecognitions == null && timer.seconds() < timeout) {
                try {
                    sleep(100);
                } catch (InterruptedException e) {
                }
                updatedRecognitions = tfod.getUpdatedRecognitions();
            }

            if (updatedRecognitions != null) {
                opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    i++;
                    opmode.telemetry.addLine()
                            .addData(String.format("label (%d)", i), recognition.getLabel())
                            .addData("Conf", "%.02f", recognition.getConfidence())
                            .addData("Loc", "(%.01f,%.01f,%.01f,%.01f)", recognition.getLeft(), recognition.getTop(), recognition.getRight(), recognition.getBottom());
                }
            } else opmode.telemetry.addData("Status", "Recognitions is NULL");
        } else opmode.telemetry.addData("Status", "TFOD is NULL");

        opmode.telemetry.update();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        if (showCamera) {
            int cameraMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        }
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (useWebCam)
            parameters.cameraName = opmode.hardwareMap.get(WebcamName.class, "Webcam 1");
        else // else assume phone back camera
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        FtcDashboard.getInstance().startCameraStream(vuforia, 0);


        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        if (showCameraOD) {
            int tfodMonitorViewId = opmode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opmode.hardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        }
        tfodParameters.minResultConfidence = 0.2f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public double getBarcodeTSE(double timeout) {

        opmode.telemetry.addData("Status", "Processing!");
        Recognition winner = null;
        if (tfod != null) {
            ElapsedTime timer = new ElapsedTime();

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = updatedRecognitions = tfod.getUpdatedRecognitions();
            ;
            while (updatedRecognitions == null && timer.seconds() < timeout) {
                try {
                    sleep(100);
                } catch (InterruptedException e) {
                }
                updatedRecognitions = tfod.getUpdatedRecognitions();
            }

            if (updatedRecognitions != null) {
                opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    i++;

                    opmode.telemetry.addLine()
                            .addData(String.format("label (%d)", i), recognition.getLabel())
                            .addData("Conf", "%.02f", recognition.getConfidence())
                            .addData("Loc", "(%.01f,%.01f,%.01f,%.01f)", recognition.getLeft(), recognition.getTop(), recognition.getRight(), recognition.getBottom());

                    if (winner == null) {
                        winner = recognition;
                    } else {
                        if (winner.getLabel().equals("Cube")) {
                            if (recognition.getLabel().equals("Cube")) {
                                if (recognition.getConfidence() >= winner.getConfidence()) {
                                    winner = recognition;
                                } else {
                                    winner = winner;
                                }
                            }
                            if (recognition.getLabel().equals("Ball")) {
                                winner = recognition;
                            }
                            if (recognition.getLabel().equals("Duck")) {
                                winner = winner;
                            }
                            if (recognition.getLabel().equals("Marker")) {
                                winner = winner;
                            }
                        } else if (winner.getLabel().equals("Ball")) {
                            if (recognition.getLabel().equals("Cube")) {
                                winner = winner;
                            }
                            if (recognition.getLabel().equals("Ball")) {
                                if (recognition.getConfidence() >= winner.getConfidence()) {
                                    winner = recognition;
                                } else {
                                    winner = winner;
                                }
                            }
                            if (recognition.getLabel().equals("Duck")) {
                                winner = winner;
                            }
                            if (recognition.getLabel().equals("Marker")) {
                                winner = winner;
                            }
                        } else if (winner.getLabel().equals("Duck")) {
                            if (recognition.getLabel().equals("Duck")) {
                                if (recognition.getConfidence() > winner.getConfidence()) {
                                    recognition = winner;
                                } else {
                                    winner = winner;
                                }

                            }
                            if (recognition.getLabel().equals("Ball")) {
                                recognition = winner;
                            }
                            if (recognition.getLabel().equals("Cube")) {
                                recognition = winner;
                            }
                            if (recognition.getLabel().equals("Marker")) {
                                winner = winner;
                            }
                        } else if (winner.getLabel().equals("Marker")) {
                            if (recognition.getLabel().equals("Marker")) {
                                if (recognition.getConfidence() > winner.getConfidence()) {
                                    recognition = winner;
                                } else {
                                    winner = winner;
                                }
                            }
                            if (recognition.getLabel().equals("Cube")) {
                                recognition = winner;
                            }
                            if (recognition.getLabel().equals("Ball")) {
                                recognition = winner;
                            }
                            if (recognition.getLabel().equals("Duck")) {
                                recognition = winner;
                            }
                        }
                    }
                }
            } else opmode.telemetry.addData("Status", "Recognitions is NULL");
        } else opmode.telemetry.addData("Status", "TFOD is NULL");
        if (winner == null || winner.getLabel().equals("Marker")) {
            returnvalue = 3;
            return returnvalue;
        } else {
            if (winner.getLeft() > 100 && winner.getLeft() < 500) {
                returnvalue = 2;
            } else if (winner.getLeft() > 500) {
                returnvalue = 3;
            } else {
                returnvalue = 1;
            }
        }

        opmode.telemetry.addData("Return Value", returnvalue);
        opmode.telemetry.update();
        return returnvalue;
    }

    public double getBarcodeDuck(double timeout) {

        opmode.telemetry.addData("Status", "Processing!");
        Recognition winner = null;
        if (tfod != null) {
            ElapsedTime timer = new ElapsedTime();

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = updatedRecognitions = tfod.getUpdatedRecognitions();
            ;
            while (updatedRecognitions == null && timer.seconds() < timeout) {
                try {
                    sleep(100);
                } catch (InterruptedException e) {
                }
                updatedRecognitions = tfod.getUpdatedRecognitions();
            }

            if (updatedRecognitions != null) {
                opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    i++;

                    opmode.telemetry.addLine()
                            .addData(String.format("label (%d)", i), recognition.getLabel())
                            .addData("Conf", "%.02f", recognition.getConfidence())
                            .addData("Loc", "(%.01f,%.01f,%.01f,%.01f)", recognition.getLeft(), recognition.getTop(), recognition.getRight(), recognition.getBottom());

                    if (winner == null) {
                        winner = recognition;
                    }
                    else {
                        if (winner.getLabel().equals("Cube")) {
                            if (recognition.getLabel().equals("Cube")) {
                                if (recognition.getConfidence() >= winner.getConfidence()) {
                                    winner = recognition;
                                } else {
                                    winner = winner;
                                }
                            }
                            if (recognition.getLabel().equals("Ball")) {
                                winner = recognition;
                            }
                            if (recognition.getLabel().equals("Duck")) {
                                winner = recognition;
                            }
                            if (recognition.getLabel().equals("Marker")) {
                                winner = winner;
                            }
                        }
                        else if (winner.getLabel().equals("Ball")) {
                            if (recognition.getLabel().equals("Cube")) {
                                winner = winner;
                            }
                            if (recognition.getLabel().equals("Ball")) {
                                if (recognition.getConfidence() >= winner.getConfidence()) {
                                    winner = recognition;
                                } else {
                                    winner = winner;
                                }
                            }
                            if (recognition.getLabel().equals("Duck")) {
                                winner = recognition;
                            }
                            if (recognition.getLabel().equals("Marker")) {
                                winner = winner;
                            }
                        }
                        else if (winner.getLabel().equals("Duck")) {
                            if (recognition.getLabel().equals("Duck")) {
                                if (recognition.getConfidence() > winner.getConfidence()) {
                                    winner = recognition;
                                } else {
                                    winner = winner;
                                }

                            }
                            if (recognition.getLabel().equals("Ball")) {
                                winner = winner;
                            }
                            if (recognition.getLabel().equals("Cube")) {
                                winner = winner;
                            }
                            if (recognition.getLabel().equals("Marker")) {
                                winner = winner;
                            }
                        }
                        else if (winner.getLabel().equals("Marker")) {
                            if (recognition.getLabel().equals("Marker")) {
                                if (recognition.getConfidence() > winner.getConfidence()) {
                                    winner = recognition;
                                } else {
                                    winner = winner;
                                }
                            }
                            if (recognition.getLabel().equals("Cube")) {
                                winner = recognition;
                            }
                            if (recognition.getLabel().equals("Ball")) {
                                winner = recognition;
                            }
                            if (recognition.getLabel().equals("Duck")) {
                                winner = recognition;
                            }
                        }
                    }
                }
            } else opmode.telemetry.addData("Status", "Recognitions is NULL");
        } else opmode.telemetry.addData("Status", "TFOD is NULL");
        if (winner == null ) {
            returnvalue = 4;
            opmode.telemetry.addData("Object is not there",returnvalue);
            return returnvalue;
        }
        else if(winner.getLabel().equals("Marker")){
            returnvalue = 5;
            opmode.telemetry.addData("Object is a marker",returnvalue);
        }
        else {
            if (winner.getLeft() > 100 && winner.getLeft() < 420) {
                returnvalue = 2;
            } else if (winner.getLeft() > 420) {
                returnvalue = 3;
            } else {
                returnvalue = 1;
            }
        }

        opmode.telemetry.addData("Return Value", returnvalue);
        opmode.telemetry.update();
        return returnvalue;
    }

    public double getBarcodeTSEUpdated(double timeout) {
        opmode.telemetry.addData("Status", "Processing!");
        Recognition winner = null;
        if (tfod != null) {
            ElapsedTime timer = new ElapsedTime();

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = updatedRecognitions = tfod.getUpdatedRecognitions();
            ;
            while (updatedRecognitions == null && timer.seconds() < timeout) {
                try {
                    sleep(100);
                } catch (InterruptedException e) {
                }
                updatedRecognitions = tfod.getUpdatedRecognitions();
            }
            if (updatedRecognitions != null) {
                opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    i++;
                    opmode.telemetry.addLine()
                            .addData(String.format("label (%d)", i), recognition.getLabel())
                            .addData("Conf", "%.02f", recognition.getConfidence())
                            .addData("Loc", "(%.01f,%.01f,%.01f,%.01f)", recognition.getLeft(), recognition.getTop(), recognition.getRight(), recognition.getBottom());

                    if(recognition.getLabel().equals("Duck")&&recognition.getBottom()>140){
                        if(winner == null||recognition.getConfidence()> winner.getConfidence()){
                            winner = recognition;
                        }
                    }
                }

            } else opmode.telemetry.addData("Status", "Recognitions is NULL");
        } else opmode.telemetry.addData("Status", "TFOD is NULL");
        if (winner == null || winner.getLabel().equals("Marker") || winner.getLabel().equals("Ball") || winner.getLabel().equals("Cube")) {
            returnvalue = 0;
            opmode.telemetry.addData("Object is not detect or object is not a duck",returnvalue);
            return returnvalue;
        } else {
            if (winner.getLeft() > 100 && winner.getLeft() < 420) {
                returnvalue = 2;
            } else if (winner.getLeft() > 420) {
                returnvalue = 3;
            } else {
                returnvalue = 1;
            }
        }

        opmode.telemetry.addData("Return Value", returnvalue);
        opmode.telemetry.update();
        return returnvalue;
    }
    public void convertBarcode() {
        if (returnvalue == 1) {
            robot.intake.setGantryPosition(Robot.ARM_LAYER1_POS,0);
        } else if (returnvalue == 2) {
            robot.intake.setGantryPosition(Robot.ARM_LAYER2_POS,0);
        } else if (returnvalue == 3) {
            robot.intake.setGantryPosition(Robot.ARM_LAYER3_POS,0);
        } else{
            telemetry.addData("No objects found in vision",404);
        }
    }
}
