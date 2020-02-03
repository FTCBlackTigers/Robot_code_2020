/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
public class SkystoneDetection {
    public enum SkystonePos {
        LEFT, CENTER, RIGHT, NONE
    }

    public static final double WAIT_FOR_DETECTION = 3;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AW/F0cP/////AAABmUpR4+dbt0Negw2nqaCH9Cw2gV4ZxuUmpeJMm7XOTdQVumthQcOeoS9qktHy4EvXtMAFoh7n5KeMiLMDqtKvd1TrbYNUy3f9ST3TMkH2hFYKB6oJMJPB8oelL9Bst/2XJBz0ycMMcKmSsIwyOqwOuHamAlwfT+o7VusfYmY7FPvnXuhn8obCeB5x0hhjwjsBuOz2wnx1us6N5y6on0rdc1DOzC2gI767QLVXAvPyJvMfgtZRGcfzFk0evuSVxrIPCRQBjQYK2s5SsBLZ4sEO4HelibKK5kg0lgT9P1uHSNa8SWX+4wpJm76dFw1nSL7YElieByOTXxycmOdhWaae5qb1lvYYmDiBNFiwfHRDkBRD";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private OpMode opMode;

    private SkystonePos lastPos = SkystonePos.NONE;

    public void init(HardwareMap hardwareMap, OpMode opMode) {
        this.opMode = opMode;

        initVuforia(hardwareMap);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        }
        if (tfod != null) {
            tfod.activate();
        }
    }

    public SkystonePos getSkystonePos() {
        if (tfod != null) {
            opMode.telemetry.addLine("SKYSTONE");
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                opMode.telemetry.addData("\t#Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 2) {
                    opMode.telemetry.addData("\tfirst label", updatedRecognitions.get(0).getLabel()).addData("second label", updatedRecognitions.get(1).getLabel());
                    if (updatedRecognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT) && updatedRecognitions.get(1).getLabel().equals(LABEL_FIRST_ELEMENT)) {
                        lastPos = SkystonePos.RIGHT;
                        return SkystonePos.RIGHT;
                    } else {
                        if (updatedRecognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            if (updatedRecognitions.get(0).getLeft() > updatedRecognitions.get(1).getLeft()) {
                                lastPos = SkystonePos.CENTER;
                                return SkystonePos.CENTER;
                            } else {
                                lastPos = SkystonePos.LEFT;
                                return SkystonePos.LEFT;
                            }
                        } else {
                            if (updatedRecognitions.get(1).getLeft() > updatedRecognitions.get(0).getLeft()) {
                                lastPos = SkystonePos.CENTER;
                                return SkystonePos.CENTER;
                            } else {
                                lastPos = SkystonePos.LEFT;
                                return SkystonePos.LEFT;
                            }
                        }
                    }
                } else {
                    lastPos = SkystonePos.NONE;
                    return SkystonePos.NONE;
                }
            }
        }
        return lastPos;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //  parameters.cameraDirection = CameraDirection.FRONT;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void shutdown(){
        if(this.tfod != null) {
            this.tfod.shutdown();
        }
    }
}
