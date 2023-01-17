
package org.firstinspires.ftc.teamcode.FTCLib.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {

    private LiftSubsystem subsystem;
    private ElapsedTime time;
    private double liftTime = 1.2;
    boolean yippee = false;


    public LiftCommand(LiftSubsystem liftSubsystem, ElapsedTime timer) {
        subsystem = liftSubsystem;
        time = timer;
    }

    @Override
    public void initialize() {
        time.reset();
        liftTime = 3.4;
        subsystem.motorUp();
    }

    @Override // tells the computer to use my code not the class im extending
    public boolean isFinished() {
        return time.seconds() >= liftTime;
    }
    @Override
    public void end(boolean interupted) {
        subsystem.motorHold();
    }
}

