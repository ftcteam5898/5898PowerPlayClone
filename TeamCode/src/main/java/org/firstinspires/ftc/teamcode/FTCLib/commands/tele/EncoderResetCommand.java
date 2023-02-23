package org.firstinspires.ftc.teamcode.FTCLib.commands.tele;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftEncoderSubsystem;

import java.util.function.BooleanSupplier;

public class EncoderResetCommand extends CommandBase {

    private LiftEncoderSubsystem encoderSubsystem;
    private boolean yippee = false;
    private boolean pain = false;

    public EncoderResetCommand(LiftEncoderSubsystem subsystem) {
        encoderSubsystem = subsystem;
    }
    public BooleanSupplier getZeroCondition() {
        final boolean zero = yippee;
        BooleanSupplier fin = () -> zero;
        return fin;
    }
    @Override
    public void initialize() {
        encoderSubsystem.resetMotor();
        yippee = true;
    }
    @Override
    public void end(boolean interupted) {
        final boolean zero = yippee;
        BooleanSupplier done = () -> zero;
    }
}
