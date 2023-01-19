package org.firstinspires.ftc.teamcode.FTCLib.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;

public class LiftStop extends CommandBase {

    private LiftSubsystem subsystem;
    private ElapsedTime time;
    private double liftTime;
    private boolean yippee = false;

    public LiftStop(LiftSubsystem liftSubsystem) {
        subsystem = liftSubsystem;
    }
    @Override
    public void initialize() {
        yippee = true;
    }
    @Override
    public boolean isFinished() {
        return yippee;
    }
    @Override
    public void end(boolean interupted) {
        subsystem.motorStop();
    }
}
