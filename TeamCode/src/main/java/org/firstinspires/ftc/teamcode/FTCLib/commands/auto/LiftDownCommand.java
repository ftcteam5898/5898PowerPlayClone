
package org.firstinspires.ftc.teamcode.FTCLib.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
// this lift command does not use PID
public class LiftDownCommand extends CommandBase {

    private LiftSubsystem subsystem;
    private ElapsedTime time;
    private double liftTime = 0.5;
    private int junction;
    // junction values: 1 = low, 2 = medium, 3 = high

    public LiftDownCommand(LiftSubsystem liftSubsystem, ElapsedTime timer) {
        subsystem = liftSubsystem;
        time = timer;
    }
    public LiftDownCommand(LiftSubsystem liftSubsystem) {
        subsystem = liftSubsystem;
    }

    @Override
    public void initialize() {
        time.reset();
        liftTime = 2.8;
        subsystem.motorDown();
    }

    @Override // tells the computer to use my code not the class im extending
    public boolean isFinished() {
        return time.seconds() >= liftTime;
    }
    @Override
    public void end(boolean interupted) {
        subsystem.motorStop();
    }
}