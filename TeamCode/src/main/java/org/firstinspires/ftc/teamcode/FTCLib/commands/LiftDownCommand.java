package org.firstinspires.ftc.teamcode.FTCLib.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
// this lift command does not use PID
public class LiftDownCommand extends CommandBase {

    private LiftSubsystem subsystem;
    private ElapsedTime time;
    private double liftTime;
    private int junction;
    // junction values: 1 = low, 2 = medium, 3 = high

    public LiftDownCommand(LiftSubsystem subsystem, ElapsedTime timer) {
        this.subsystem = subsystem;
        addRequirements(this.subsystem);
        time = timer;
    }
    @Override
    public void initialize() {
        time.reset();
        subsystem.motorDown();
        liftTime = 0.02;
    }
    @Override // tells the computer to use my code not the class im extending
    public boolean isFinished() {
        return time.seconds() >= liftTime;
    }
}