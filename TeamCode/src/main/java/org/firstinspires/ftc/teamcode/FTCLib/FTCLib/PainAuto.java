package org.firstinspires.ftc.teamcode.FTCLib.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.FTCLib.commands.auto.MonkeCommandsLeft;
import org.firstinspires.ftc.teamcode.FTCLib.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.IntSupplier;

@Autonomous(name = "PainAuto")

public class PainAuto extends CommandOpMode {
    private Blinker control_Hub;
    private DcMotor lb, rb, lf, rf;

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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        // motors (may change later for different motor names) ;-;
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");

        // motor spin direction (may change later) ;-;
        lb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);

        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == left || tag.id == middle || tag.id == right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("We got em' boys\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Target not sighted :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the target before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Target not sighted :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
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
        if (opModeIsActive()) {
            if (tagID.getAsInt() == 1) {
                // go forward 1 tile stafe left 1 tile
                forwardAmt(-1100, .3);
                strafe(1300, .2);
                forwardAmt(-400, .3);
                stopRobot();
            } else if (tagID.getAsInt() == 2) {


                // go forward 1 tile
                forwardAmt(-1500, .3);
                stopRobot();

            } else {
                // go forward 1 tile, strafe right 1 tile
                //forwardAmt(1400, .3);
                forwardAmt(-1100, .3);
                strafe(-1400, .2);
                forwardAmt(-400, .3);
                stopRobot();
            }

        }
    }


    public void forwardAmt(int amt, double power) {
        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() + amt);

        runToPosition();

        while (opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy())) {
            //telemetry.addData("Mode", "running");
            //telemetry.addData("LB", lb.getCurrentPosition());
            //telemetry.addData("LF", lf.getCurrentPosition());
            telemetry.addData("RB", rb.getCurrentPosition());
            telemetry.addData("RF", rf.getCurrentPosition());
            telemetry.addData("RB Power", rb.getPower());
            telemetry.addData("RF Power", rf.getPower());
            telemetry.addData("LB", lb.getCurrentPosition());
            telemetry.addData("LF", lf.getCurrentPosition());
            telemetry.addData("LB Power", lb.getPower());
            telemetry.addData("LF Power", lf.getPower());


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


    public void rightAmt(int amt, double power) {
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

    public void leftAmt(int amt, double power) {
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

    public void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

        
