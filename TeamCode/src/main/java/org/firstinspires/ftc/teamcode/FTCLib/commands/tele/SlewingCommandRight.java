package org.firstinspires.ftc.teamcode.FTCLib.commands.tele;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.SpinSubsystem;

public class SlewingCommandRight extends CommandBase {

    private SpinSubsystem spinSubsystem;

    public SlewingCommandRight(SpinSubsystem subsystem) {
        spinSubsystem = subsystem;
    }
    @Override
    public void initialize() {
        spinSubsystem.rotateRight();
    }
    @Override
    public void end(boolean interupted) {
        spinSubsystem.holdSpin();
    }
}
