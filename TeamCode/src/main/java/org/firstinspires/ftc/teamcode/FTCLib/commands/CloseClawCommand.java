package org.firstinspires.ftc.teamcode.FTCLib.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;

public class CloseClawCommand extends CommandBase {

    private ClawSubsystem subsystem;

    public CloseClawCommand(ClawSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(this.subsystem);
    }
    @Override
    public void initialize() {
        this.subsystem.closeClaw();
    }
}