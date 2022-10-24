package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
@Autonomous(name = "Auto_Red_V1", group = "Auto_Red")

public class Auto_Red_V1 extends LinearOpMode
{
    private Blinker control_Hub;
    private DcMotor lb, rb, lf, rf;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

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
    "AeOuPaX/////AAABmebKpI9d8EyFsdPTMDjvCkB0wFESJpSWHlnyJ8jXj3U1/pY/+rWqhr36k9GxU+6BuK0+qloTKs7VV1R8l+cRgcooB/qBIqmAUnfH49n3TgXoUXCClsbQ2gYEOeINowutk2vkpurFU8f+QGUuI038Pcb7av2eCMvgi8oAdufjhBMfjQ57s79F3HbqQD758kak4R7x8Fbj3k32cotjSxsziZ7N0aNPUdZy/v2GX7MKu1ZGdqrDzOkR/ClVPnwIXkF27VOseqeHJQavMg2H5M90zABYnS9C5fEDwVYmIwsGLhoYsRSEFHqiio1lVhhtDHFmoEJnQQ3VuDP5xwWpv3SyTNwXOFyV9xnJL6r3Q12itF71";

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

    @Override
    public void runOpMode() {
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

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        // motors (may change later for different motor names) ;-;
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");

        // motor spin direction (may change later) ;-;
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        if (opModeIsActive()) {
            int counter = 0; // loop control variable so robot doesn't search forever
            String label = ""; // variable to change what label is detected
            double conf = 0.0; // keeps up with confidence of reading
            while (opModeIsActive()) {
                counter++;
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            // stores label that is being recognized with the highest confidence
                            if (label != recognition.getLabel() && conf < recognition.getConfidence()) {
                                label = recognition.getLabel();
                                conf = recognition.getConfidence();
                            }
                        }
                        telemetry.addData("Label stored:", "%s %.2f", label, conf); // showing the label and confidence
                        telemetry.update();
                    }
                }
                if (counter >= 5000)
                {
                    break;
                }
            }
            if (label.equals("1 Bolt"))
            {
                // go forward 1 tile stafe left 1 tile
                forwardAmt(1000, .6);
                strafe(-1200, .4);
            }
            else if (label.equals("2 Bulb"))
            {
                // go forward 1 tile
                forwardAmt(1000, .6);
            }
            else
            {
                // go forward 1 tile, strafe right 1 tile
                forwardAmt(1000, .6);
                strafe(1200, .4);
            }
        }
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
    public void forwardAmt(int amt, double power) {

        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() + amt);

        runToPosition();

        while (opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy())) {
            telemetry.addData("Mode", "running");
            telemetry.update();
            allPower(power);
        }

        stopRobot();
        runUsingEncoder();

    }



    public void backAmt(int amt, double power) {

        rf.setTargetPosition(rf.getCurrentPosition() - amt);
        rb.setTargetPosition(rb.getCurrentPosition() - amt);
        lf.setTargetPosition(lf.getCurrentPosition() - amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);

        runToPosition();



        while (opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy())) {
            telemetry.addData("Mode", "running");
            telemetry.update();
            allPower(power);
        }

        stopRobot();
        runUsingEncoder();
    }



    public void rightAmt(int amt, double power)
    {
        rf.setTargetPosition(rf.getCurrentPosition() - amt);
        rb.setTargetPosition(rb.getCurrentPosition() - amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() + amt);

        runToPosition();

        while (opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy())) {

            telemetry.addData("Mode", "running");
            telemetry.update();
            allPower(power);
        }

        stopRobot();
        runUsingEncoder();

    }

    public void leftAmt(int amt, double power)
    {
        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() - amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);

        runToPosition();



        while (opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy())) {
            telemetry.addData("Mode", "running");
            telemetry.update();
            allPower(power);
        }

        stopRobot();
        runUsingEncoder();

    }



    public void strafe(int amt, double power) {

        rf.setTargetPosition(rf.getCurrentPosition() - amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);

        runToPosition();



        while (opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy())) {
            telemetry.addData("Mode", "running");
            telemetry.update();
            allPower(power);
        }

        stopRobot();
        runUsingEncoder();
    }



    // set mode for all motors

    public void runUsingEncoder() {

        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void runToPosition() {

        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    // stop all movement

    public void stopRobot() {

        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }

    // set all power at once
    public void allPower(double power) {

        rf.setPower(power);
        rb.setPower(power);
        lf.setPower(power);
        lb.setPower(power);
    }
}


