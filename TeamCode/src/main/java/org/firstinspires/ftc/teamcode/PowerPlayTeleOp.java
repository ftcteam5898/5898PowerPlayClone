package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class PowerPlayTeleOp extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    private boolean FIELD_CENTRIC = false;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor slide, spin;
    private DcMotorEx launcher2;
    private Servo claw;
    private int counter =0;
    public boolean bool = false;
    public boolean bool2 = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "lf", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rf", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "lb", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rb", Motor.GoBILDA.RPM_312)
        );

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slide = hardwareMap.get(DcMotor.class, "slide");
        spin = hardwareMap.get(DcMotor.class, "spin");
        claw = hardwareMap.get(Servo.class, "claw");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(10);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx driverOp2 = new GamepadEx(gamepad2);

        waitForStart();
        slide.setPower(0.7);
        int position = 0;

        while (!isStopRequested()) {
            telemetry.addData("Field Centric?", FIELD_CENTRIC);

            if (gamepad1.a) FIELD_CENTRIC = true;
            if (gamepad1.b) FIELD_CENTRIC = false;

            telemetry.addData("servo", claw.getDirection());
            if(gamepad2.dpad_up && bool2 == true)
            {
                bool2 = false;
                bool = true;
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setTargetPosition(10);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(bool == true)
            {
                if (gamepad2.a && slide.getCurrentPosition() < -200) {

                    slide.setTargetPosition(slide.getCurrentPosition()+200);
                }
                else if (gamepad2.b && slide.getCurrentPosition() > -4400) {

                    slide.setTargetPosition(slide.getCurrentPosition()-200);
                }
            }
            else
            {
                slide.setPower(0.2);
                if (gamepad2.a) {
                    slide.setTargetPosition(slide.getCurrentPosition()+200);
                }
                else if (gamepad2.b) {
                    slide.setTargetPosition(slide.getCurrentPosition()-200);
                }
            }

            if(gamepad2.right_bumper) {
                spin.setPower(0.4);
            }
            else if(gamepad2.left_bumper) {
                spin.setPower(-0.4);
            }
            else {
                spin.setPower(0);
            }


            if(gamepad2.y){
                claw.setPosition(50);
            }
            else if(gamepad2.x) {
                claw.setPosition(-60);
            }

            // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
            // These are related to the left stick x value, left stick y value, and
            // right stick x value respectively. These values are passed in to represent the
            // strafing speed, the forward speed, and the turning speed of the robot frame
            // respectively from [-1, 1].

            if (!FIELD_CENTRIC) {

                // For a robot centric model, the input of (0,1,0) for (leftX, leftY, rightX)
                // will move the robot in the direction of its current heading. Every movement
                // is relative to the frame of the robot itself.
                //
                //                 (0,1,0)
                //                   /
                //                  /
                //           ______/_____
                //          /           /
                //         /           /
                //        /___________/
                //           ____________
                //          /  (0,0,1)  /
                //         /     ↻     /
                //        /___________/

                // optional fourth parameter for squared inputs
                drive.driveRobotCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY()*-1,
                gamepad1.right_trigger - gamepad1.left_trigger,
                        false
                );
            } else {

                // Below is a model for how field centric will drive when given the inputs
                // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
                // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
                // regardless of the heading.
                //
                //                   heading
                //                     /
                //            (0,1,0) /
                //               |   /
                //               |  /
                //            ___|_/_____
                //          /           /
                //         /           / ---------- (1,0,0)
                //        /__________ /

                // optional fifth parameter for squared inputs
                drive.driveFieldCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY()*-1,
                        gamepad1.right_trigger - gamepad1.left_trigger,
                        imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
                telemetry.update();
            }

        }
    }

}

/*





@TeleOp(name="Meet 5", group="Iterative Opmode")

public class Meet5Tele extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lb, lf, rb, rf, slide, spin;
    private Servo claw;
    private int counter = 0;
    public boolean bool = false;
    public boolean bool2 = true;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        int soundID = hardwareMap.appContext.getResources().getIdentifier("imposters among", "raw", hardwareMap.appContext.getPackageName());

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        slide = hardwareMap.get(DcMotor.class, "slide");
        spin = hardwareMap.get(DcMotor.class, "spin");
        claw = hardwareMap.get(Servo.class, "claw");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(10);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry


        double r = Math.hypot(gamepad1.right_stick_x - gamepad1.left_stick_x, gamepad1.right_stick_y - gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y - gamepad1.left_stick_y, -gamepad1.right_stick_x + gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_trigger - gamepad1.left_trigger;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        lf.setPower(-v1*0.7);
        rf.setPower(-v2*0.7);
        lb.setPower(-v3*0.7);
        rb.setPower(-v4*0.7);
        slide.setPower(0.7);
        int position = 0;


        //duck wheel on a / b press


        // if (gamepad2.x){
        //   SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        //}

        telemetry.addData("servo", claw.getDirection());
        if(gamepad2.dpad_up && bool2 == true)
        {
            bool2 = false;
            bool = true;
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setTargetPosition(10);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(bool == true)
        {
            if (gamepad2.a && slide.getCurrentPosition() < -200) {

                slide.setTargetPosition(slide.getCurrentPosition()+200);
            }
            else if (gamepad2.b && slide.getCurrentPosition() > -4400) {

                slide.setTargetPosition(slide.getCurrentPosition()-200);
            }
        }
        else
        {
            slide.setPower(0.2);
            if (gamepad2.a) {
                slide.setTargetPosition(slide.getCurrentPosition()+200);
            }
            else if (gamepad2.b) {
                slide.setTargetPosition(slide.getCurrentPosition()-200);
            }
        }

        if(gamepad2.right_bumper) {
            spin.setPower(0.4);
        }
        else if(gamepad2.left_bumper) {
            spin.setPower(-0.4);
        }
        else {
            spin.setPower(0);
        }


        if(gamepad2.y){
            claw.setPosition(50);
        }
        else if(gamepad2.x) {
            claw.setPosition(-60);
        }


    }

}

 */