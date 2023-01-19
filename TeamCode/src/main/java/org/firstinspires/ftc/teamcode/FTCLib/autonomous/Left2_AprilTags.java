
package org.firstinspires.ftc.teamcode.FTCLib.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTCLib.commands.GorillaCommandGroupLeft;
import org.firstinspires.ftc.teamcode.FTCLib.commands.GorillaCommandGroupRight;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.SpinSubsystem;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.RoadRunnerDooDoo.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.function.IntSupplier;

@Autonomous(name = "LeagueTournamentLeft", group = "FTCLib")
public class Left2_AprilTags extends CommandOpMode {
    private ElapsedTime time;

    // hardware declaration
    private SimpleServo clawServo;
    private Motor liftMotor;
    private Motor spinMotor;

    // subsystem declaration
    private MecanumDriveSubsystem driveSubsystem;
    private LiftSubsystem liftSubsystem;
    private ClawSubsystem clawSubsystem;
    private SpinSubsystem spinSubsystem;

    // camera & CV
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int left = 1;
    int middle = 2;
    int right = 3;

    AprilTagDetection tagOfInterest = null;


    @Override
    public void initialize() {
        // hardware initialization
        clawServo = new SimpleServo(hardwareMap, "claw", -90, 90);
        liftMotor = new Motor(hardwareMap, "slide");
        spinMotor = new Motor(hardwareMap, "spin");

        // subsystem initialization
        driveSubsystem = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        liftSubsystem = new LiftSubsystem(liftMotor);
        clawSubsystem = new ClawSubsystem(clawServo);
        spinSubsystem = new SpinSubsystem(spinMotor);


        //imageRec = new ImageRecognitionCommand(time);
        time = new ElapsedTime();

        schedule(new InstantCommand(() -> clawSubsystem.openClaw()));
        schedule(new InstantCommand(() -> spinSubsystem.holdSpin()));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("We got em' boys ;)\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Target not sighted :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the target before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Target not sighted :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        int tagForCommand = 0;
        if (tagOfInterest == null) tagForCommand = 1;
        else tagForCommand = tagOfInterest.id;
        final int tagNum = tagForCommand;
        IntSupplier tagID = () -> tagNum;

        telemetry.addData("Tag identified:", tagForCommand);
        telemetry.update();

        schedule(new WaitUntilCommand(this::isStarted)
                .andThen(new InstantCommand(() -> clawSubsystem.closeClaw()))
                .andThen(new GorillaCommandGroupLeft(driveSubsystem, liftSubsystem, clawSubsystem, tagID)));
    }

    public void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}