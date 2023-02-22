package org.firstinspires.ftc.teamcode.FTCLib.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {

    private LiftSubsystem subsystem;

    public LiftCommand(LiftSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(this.subsystem);
    }
    @Override
    public void initialize() {this.subsystem.motorUp();}

}
