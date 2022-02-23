package org.firstinspires.ftc.teamcode.RobotClasses.Subsytems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class TFOD {

    TankDrive drivetrain = new TankDrive();

    HardwareMap hardwareMap = null;

    private static String object = null;

    private static double currentLeft = 0, currentRight = 0, currentTop = 0, currentBottom = 0;

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     */

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball", "Cube", "Duck", "Marker"};

    private static final String VUFORIA_KEY = "Acy/Mw7/////AAABmVeoJmayA0LFsPI//25wXiAgKvF8A1IMKKPVnU/YmD2SLqIfVxja/iMw9FGXlbh6ipPCe1BFslVFQra+jdueadfzzLqYiH9I9BAz0gOjuhBQB3bxgRnHI4nFWwaaRd0BYN0MYlgXZCcLjL5YdP/dnk3ffrlMlf4U5IK/yJpsxw6Eum84uoiFq7gnyOAcdJR2zXpF86/e/L0iPQrloOtqspc5Pp4u1ra2e6Sa3fWhHbB2g4wC4nYgi980JBGxZdvL1tkVoMqmbvrylRwF4Jsm7NsDKLjkZRnGULl/xXSFJ9ry45RAEGxh745HQRiJ2/lNpdabZpXONPi5xMscczlGvUpgCNebeumlqXrA7swmEsGE";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    TFObjectDetector tfod = null;


    public void findObject(String targetObject, double targetLeft, double targetRight, double targetTop, double targetBottom) {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
    if (tfod != null) {
        tfod.activate();

        //magnification, some other weird thing dont change it(specific to each thing)
        tfod.setZoom(1, 16.0 / 9.0);

        if (tfod != null){
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    object = recognition.getLabel();
                    if (object == targetObject){
                       currentLeft = recognition.getLeft();
                       currentRight = recognition.getRight();
                       currentTop = recognition.getTop();
                       currentBottom = recognition.getBottom();
                       if (currentTop > targetTop){
                           drivetrain.drive(targetTop - currentTop, targetTop - currentTop, 360);
                       }
                    }
                i++;
                }
            }
        }
    }
            /*

telemetry stuff

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }
*/
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
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
}

