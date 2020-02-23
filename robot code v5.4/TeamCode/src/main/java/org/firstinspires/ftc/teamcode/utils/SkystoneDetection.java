package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


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
                    double skystoneLeft = -1;
                    double stoneLeft = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            skystoneLeft = recognition.getLeft();
                        } else if (stoneLeft == -1) {
                            stoneLeft = recognition.getLeft();
                        } else {
                            lastPos = SkystonePos.RIGHT;
                            return SkystonePos.RIGHT;
                        }
                    }
                    opMode.telemetry.addData("Skystone Left", skystoneLeft + ",stone left", stoneLeft);
                    opMode.telemetry.update();
                    if (skystoneLeft != -1 && stoneLeft != -1) {
                        if (skystoneLeft > stoneLeft) {
                            lastPos = SkystonePos.CENTER;
                            return SkystonePos.CENTER;
                        } else {
                            lastPos = SkystonePos.LEFT;
                            return SkystonePos.LEFT;
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
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void shutdown() {
        if (this.tfod != null) {
            this.tfod.shutdown();
        }
    }
}