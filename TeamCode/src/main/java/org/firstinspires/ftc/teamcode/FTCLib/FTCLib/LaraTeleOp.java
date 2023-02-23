package org.firstinspires.ftc.teamcode.FTCLib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCLib.commands.TurnCommand;
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
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
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
    private MecanumDriveSubsystem mecanumDriveSubsystem;

    private EncoderResetCommand resetCommand;
    private DriveCommand driveCommand;

    private GamepadEx Dylan, Scott;
    private GamepadKeys.Trigger left, right;
    private RevIMU revIMU;
    private ElapsedTime time;

    private final double driveMult = 1.0;

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

        lb.motor.setDirection(DcMotor.Direction.FORWARD);
        lf.motor.setDirection(DcMotor.Direction.FORWARD);

        clawSubsystem = new ClawSubsystem(clawServo);
        liftSubsystem = new LiftSubsystem(liftMotor, 1);
        spinSubsystem = new SpinSubsystem(spinMotor);
        encoderSubsystem = new LiftEncoderSubsystem(liftMotor);
        driveSubsystem = new DriveSubsystem(lf, rf, lb, rb);

        resetCommand = new EncoderResetCommand(encoderSubsystem);

        Dylan = new GamepadEx(gamepad1);
        Scott = new GamepadEx(gamepad2);

        // maybe add threshold to account for possible drift
        double turn  = 0;
        if (Dylan.getRightX() != 0) turn = Dylan.getRightX();
        else turn = -1 * Dylan.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) +
                Dylan.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        final double turnVal = turn;
        DoubleSupplier rightX = () -> turnVal;


        driveCommand = new DriveCommand(driveSubsystem, Dylan::getLeftX, Dylan::getLeftY,
                rightX);

        time = new ElapsedTime();

        ButtonReader breader = new ButtonReader(Scott, GamepadKeys.Button.B);
        ButtonReader areader = new ButtonReader(Scott, GamepadKeys.Button.A);
        ButtonReader xreader = new ButtonReader(Scott, GamepadKeys.Button.X);
        ButtonReader rightreader = new ButtonReader(Scott, GamepadKeys.Button.RIGHT_BUMPER);
        ButtonReader leftreader = new ButtonReader(Scott, GamepadKeys.Button.LEFT_BUMPER);
        ButtonReader dpadreader = new ButtonReader(Scott, GamepadKeys.Button.DPAD_UP);
        TriggerReader leftTReader = new TriggerReader(Dylan, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerReader rightTReader = new TriggerReader(Dylan, GamepadKeys.Trigger.RIGHT_TRIGGER);

        BooleanSupplier zeroCondition = resetCommand.getZeroCondition();

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
                .whenHeld(new EncoderLiftCommand(encoderSubsystem, zeroCondition.getAsBoolean()));
        Scott.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new EncoderLiftDownCommand(encoderSubsystem, zeroCondition.getAsBoolean()));
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
