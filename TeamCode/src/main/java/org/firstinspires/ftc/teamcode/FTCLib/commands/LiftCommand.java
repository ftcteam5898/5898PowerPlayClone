
package org.firstinspires.ftc.teamcode.FTCLib.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {

    private LiftSubsystem subsystem;
    private ElapsedTime time;
    private double liftTime = 0.02;
    boolean yippee = false;


    public LiftCommand(LiftSubsystem liftSubsystem, ElapsedTime timeLift) {
        subsystem = liftSubsystem;
        time = timeLift;
    }

    @Override
    public void initialize() {
        time.reset();
        subsystem.motorUp();
        liftTime = 0.02;
    }

    @Override // tells the computer to use my code not the class im extending
    public boolean isFinished() {
        if (time.seconds() >= liftTime) {
            yippee = true;
        }
        //return time.seconds() >= liftTime;
        return yippee;
    }
}

