
package org.firstinspires.ftc.teamcode.FTCLib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.FTCLib.commands.redmovement.redright.MonkeCommands1;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.RoadRunnerDooDoo.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Red_Right1", group = "FTCLib_Red")
public class Red_Right1 extends CommandOpMode {
    private ElapsedTime time;

    // hardware declaration
    private SimpleServo clawServo;
    private Motor liftMotor;

    // subsystem declaration
    private MecanumDriveSubsystem driveSubsystem;
    private LiftSubsystem liftSubsystem;
    private ClawSubsystem clawSubsystem;

    // image recognition declaration
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "purpleDuck",
            "yellowDuck",
            "mallard"
    };
    private static final String VUFORIA_KEY =
            "AeOuPaX/////AAABmebKpI9d8EyFsdPTMDjvCkB0wFESJpSWHlnyJ8jXj3U1/pY/+rWqhr36k9GxU+6BuK0+qloTKs7VV1R8l+cRgcooB/qBIqmAUnfH49n3TgXoUXCClsbQ2gYEOeINowutk2vkpurFU8f+QGUuI038Pcb7av2eCMvgi8oAdufjhBMfjQ57s79F3HbqQD758kak4R7x8Fbj3k32cotjSxsziZ7N0aNPUdZy/v2GX7MKu1ZGdqrDzOkR/ClVPnwIXkF27VOseqeHJQavMg2H5M90zABYnS9C5fEDwVYmIwsGLhoYsRSEFHqiio1lVhhtDHFmoEJnQQ3VuDP5xwWpv3SyTNwXOFyV9xnJL6r3Q12itF71";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    double conf;

    public String label = "";
    public String plzhelpme = "";

    public void setLabel(String help) {
        help = label;
    }

    @Override
    public void initialize() {
        initVuforia();
        initTfod();
        // hardware initialization
        clawServo = new SimpleServo(hardwareMap, "claw", -90, 90);
        liftMotor = new Motor(hardwareMap, "slide");

        // subsystem initialization
        driveSubsystem = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        liftSubsystem = new LiftSubsystem(liftMotor);
        clawSubsystem = new ClawSubsystem(clawServo);

        //imageRec = new ImageRecognitionCommand(time);
        time = new ElapsedTime();

        // image recognition stuff
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.15, 16.0 / 9.0);
        }
        {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# camera is operating'", updatedRecognitions.size());
                    // displaying the recognitions on the driver hub
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());
                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)",
                                recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        // stores label that is being recognized with the highest confidence
                        if (label != recognition.getLabel() && conf < recognition.getConfidence()) {
                            label = recognition.getLabel();
                            conf = recognition.getConfidence();
                            //plzhelpme = label;
                        }
                        if (time.seconds() >= 10.0) {
                            plzhelpme = label;
                            telemetry.addData("I'm in pain", time);
                            break;
                        }
                    }
                }
            }
            telemetry.addData("Label stored:", "%s %.2f", label, conf); // showing the label and confidence
            telemetry.update();

            schedule(new InstantCommand(() -> clawSubsystem.openClaw()));

            schedule(new RunCommand(() -> {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# camera is operating'", updatedRecognitions.size());
                        // displaying the recognitions on the driver hub
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());
                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)",
                                    recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            // stores label that is being recognized with the highest confidence
                            if (label != recognition.getLabel() && conf < recognition.getConfidence()) {
                                label = recognition.getLabel();
                                conf = recognition.getConfidence();
                                //plzhelpme = label;
                            }
                            if (time.seconds() >= 10.0) {
                                plzhelpme = label;
                                telemetry.addData("I'm in pain", time);
                                break;
                            }
                        }
                    }
                }
                telemetry.addData("Label stored:", "%s %.2f", label, conf); // showing the label and confidence
                telemetry.update();
            }
            ));
            schedule(new RunCommand(() -> {
                telemetry.addData("ligma", label);
                telemetry.addData("TEST", "WORK");
                telemetry.addData("time", time);
                telemetry.update();
            }
            ));
            schedule(new WaitUntilCommand(this::isStarted)
                    .andThen(new InstantCommand(() -> clawSubsystem.closeClaw()))
                    .andThen(new MonkeCommands1(driveSubsystem, liftSubsystem, clawSubsystem, 1)));
        }
    }
        private void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.75f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 300;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        }
        private void initVuforia () {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }
    }


