
package org.firstinspires.ftc.teamcode.FTCLib.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.SpinSubsystem;

public class LiftCommand extends CommandBase {

    private LiftSubsystem subsystem;
    private ElapsedTime time;
    private double liftTime = 1.2;
    boolean yippee = false;


    public LiftCommand(LiftSubsystem liftSubsystem, ElapsedTime timer, double timeToLift) {
        subsystem = liftSubsystem;
        time = timer;
        liftTime = timeToLift;
    }
    public LiftCommand(LiftSubsystem liftSubsystem) {
        subsystem = liftSubsystem;

    }
    public LiftCommand(LiftSubsystem liftSubsystem, ElapsedTime timer) {
        subsystem = liftSubsystem;
        time = timer;
    }

    @Override
    public void initialize() {
        time.reset();
        // 3.4 top junction estimate 0.42 first cone estimate
        subsystem.motorUp();
    }

    @Override // tells the computer to use my code not the class im extending
    public boolean isFinished() {
        return time.seconds() >= liftTime;
    }
    @Override
    public void end(boolean interupted) {
        if (liftTime >= 2.0) {
            subsystem.highMotorHold();
        }
        else {
            subsystem.motorHold();
        }
    }
}

