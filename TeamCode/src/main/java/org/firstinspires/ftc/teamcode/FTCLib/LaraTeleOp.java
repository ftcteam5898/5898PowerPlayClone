package org.firstinspires.ftc.teamcode.FTCLib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCLib.commands.auto.CloseClawCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.auto.OpenClawCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.tele.DriveCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.tele.EncoderLiftCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.tele.EncoderLiftDownCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.tele.EncoderResetCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.tele.SlewingCommandLeft;
import org.firstinspires.ftc.teamcode.FTCLib.commands.tele.SlewingCommandRight;
import org.firstinspires.ftc.teamcode.FTCLib.commands.tele.TeleLiftCommand;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftEncoderSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.SpinSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@TeleOp(name = "LaraTeleOp", group = "FTCLib TeleOp")
public class LaraTeleOp extends CommandOpMode {

    private Motor lb, lf, rb, rf;
    private Motor liftMotor;
    private Motor spinMotor;

    private SimpleServo clawServo;

    private LiftSubsystem liftSubsystem;
    private ClawSubsystem clawSubsystem;
    private SpinSubsystem spinSubsystem;
    private LiftEncoderSubsystem encoderSubsystem;
    private DriveSubsystem driveSubsystem;

    private EncoderResetCommand resetCommand;
    private DriveCommand driveCommand;

    private GamepadEx Dylan, Scott;
    private RevIMU revIMU;
    private ElapsedTime time;

    private final double driveMult = 1.0;

    private boolean setZero = true;
    private boolean isSetZero = true;

    @Override
    public void initialize() {

        lb = new Motor(hardwareMap, "lb");
        lf = new Motor(hardwareMap, "lf");
        rb = new Motor(hardwareMap, "rb");
        rf = new Motor(hardwareMap, "rf");
        clawServo = new SimpleServo(hardwareMap, "claw",-90, 90);
        liftMotor = new Motor(hardwareMap, "slide");
        spinMotor = new Motor(hardwareMap, "spin");
        revIMU = new RevIMU(hardwareMap);
        revIMU.init();

        //rf.motor.setDirection(DcMotor.Direction.REVERSE);
        //rb.motor.setDirection(DcMotor.Direction.REVERSE);
        lb.motor.setDirection(DcMotor.Direction.FORWARD);
        lf.motor.setDirection(DcMotor.Direction.FORWARD);
        /*
        double r = Math.hypot(gamepad1.left_stick_x - gamepad1.right_stick_x, gamepad1.left_stick_y - gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y - gamepad1.right_stick_y, -gamepad1.left_stick_x + gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_trigger - gamepad1.left_trigger;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        lf.set(v1);
        rf.set(-v2);
        lb.set(v3);
        rb.set(-v4);

         */

        clawSubsystem = new ClawSubsystem(clawServo);
        liftSubsystem = new LiftSubsystem(liftMotor, 1);
        spinSubsystem = new SpinSubsystem(spinMotor);
        encoderSubsystem = new LiftEncoderSubsystem(liftMotor);
        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb, revIMU);

        /*
        rf.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         */
        resetCommand = new EncoderResetCommand(encoderSubsystem);

        Dylan = new GamepadEx(gamepad1);
        Scott = new GamepadEx(gamepad2);

        driveCommand = new DriveCommand(driveSubsystem, Dylan::getLeftX, Dylan::getLeftY,
                Dylan::getRightX, driveMult);

        time = new ElapsedTime();

        ButtonReader breader = new ButtonReader(Scott, GamepadKeys.Button.B);
        ButtonReader areader = new ButtonReader(Scott, GamepadKeys.Button.A);
        ButtonReader xreader = new ButtonReader(Scott, GamepadKeys.Button.X);
        ButtonReader rightreader = new ButtonReader(Scott, GamepadKeys.Button.RIGHT_BUMPER);
        ButtonReader leftreader = new ButtonReader(Scott, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader dpadreader = new ButtonReader(Scott, GamepadKeys.Button.DPAD_UP);

        BooleanSupplier zeroCondition = resetCommand.getZeroCondition();
        BooleanSupplier stopLift = () -> breader.wasJustReleased();

        Scott.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(new OpenClawCommand(clawSubsystem), new CloseClawCommand(clawSubsystem));
        // x claw
        // lift y
        Scott.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new TeleLiftCommand(liftSubsystem, time));
        /*
        Scott.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new LiftCommand(liftSubsystem, time), breader.wasJustReleased());
        Scott.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new LiftDownCommand(liftSubsystem, time), areader.wasJustReleased());
         */
        Scott.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new EncoderLiftCommand(encoderSubsystem, isSetZero));
        Scott.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new EncoderLiftDownCommand(encoderSubsystem, isSetZero));
        Scott.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new EncoderResetCommand(encoderSubsystem));

        // manual lift b up a down
        // spinner bumpers
        Scott.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new SlewingCommandRight(spinSubsystem), rightreader.wasJustReleased());
        Scott.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new SlewingCommandLeft(spinSubsystem), leftreader.wasJustReleased());

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }

}
