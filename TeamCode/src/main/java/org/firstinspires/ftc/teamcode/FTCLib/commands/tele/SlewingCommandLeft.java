package org.firstinspires.ftc.teamcode.FTCLib.commands.tele;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.SpinSubsystem;

public class SlewingCommandLeft extends CommandBase {

    private SpinSubsystem spinSubsystem;

    public SlewingCommandLeft(SpinSubsystem subsystem) {
        spinSubsystem = subsystem;
    }
    @Override
    public void initialize() {
        spinSubsystem.rotateLeft();
    }
    @Override
    public void end(boolean interupted) {
        spinSubsystem.holdSpin();
    }
}
