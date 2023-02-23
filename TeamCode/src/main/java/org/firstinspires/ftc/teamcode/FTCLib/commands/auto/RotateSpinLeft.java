package org.firstinspires.ftc.teamcode.FTCLib.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.SpinSubsystem;

public class RotateSpinLeft extends CommandBase {

    private SpinSubsystem subsystem;
    private ElapsedTime time;
    private double spinTime = 0.0;

    public RotateSpinLeft(SpinSubsystem spinSubsystem, ElapsedTime timer, double timeSpin) {
        subsystem = spinSubsystem;
        time = timer;
        spinTime = timeSpin;
    }

    @Override
    public void initialize() {
        time.reset();
        subsystem.rotateLeft();
    }
    @Override
    public boolean isFinished() {
        return time.seconds() >= spinTime;
    }
    @Override
    public void end(boolean interupted) {
        subsystem.holdSpin();
    }
}

