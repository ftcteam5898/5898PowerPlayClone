package org.firstinspires.ftc.teamcode.FTCLib.commands.tele;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftEncoderSubsystem;

import java.util.function.BooleanSupplier;

public class EncoderLiftCommand extends CommandBase {

    private LiftEncoderSubsystem encoderSubsystem;
    private Motor liftMotor;
    boolean yippee = false;
    private Telemetry tele;

    public EncoderLiftCommand(LiftEncoderSubsystem subsystem, boolean zero) {
        encoderSubsystem = subsystem;
        yippee = zero;
        liftMotor = encoderSubsystem.getLiftMotor();

    }
    @Override
    public void initialize() {
        if (yippee) {
            encoderSubsystem.motorUp();
        }
        else {
            encoderSubsystem.motorUpSlow();
        }
    }
    @Override
    public boolean isFinished() {
        return liftMotor.getCurrentPosition() <= -3000;
    }
    @Override
    public void end(boolean interupted) {
        encoderSubsystem.motorHold();
    }
}
