package org.firstinspires.ftc.teamcode.FTCLib.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.SpinSubsystem;

public class HoldSpinMotor extends CommandBase {

    private SpinSubsystem subsystem;
    int yippee = 0;

    public HoldSpinMotor(SpinSubsystem spinSubsystem) {
        subsystem = spinSubsystem;
    }

    @Override
    public void initialize() {
        yippee = 1;
    }
    @Override
    public boolean isFinished() {
        return yippee == 1;
    }
    @Override
    public void end(boolean interupted) {
        subsystem.holdSpin();
    }
}
