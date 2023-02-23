package org.firstinspires.ftc.teamcode.FTCLib.commands.tele;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftEncoderSubsystem;

public class EncoderLiftDownCommand extends CommandBase {

    private LiftEncoderSubsystem encoderSubsystem;
    private Motor liftMotor;
    boolean yippee = false;

    public EncoderLiftDownCommand(LiftEncoderSubsystem subsystem, boolean zero) {
        encoderSubsystem = subsystem;
        yippee = zero;
        liftMotor = encoderSubsystem.getLiftMotor();
    }
    @Override
    public void initialize() {
        if (yippee) {
            encoderSubsystem.motorDown();
        }
        else {
            encoderSubsystem.motorDownSlow();
        }
    }
    @Override
    public boolean isFinished() {
        return liftMotor.getCurrentPosition() >= -200;
    }
    @Override
    public void end(boolean interupted) {
        encoderSubsystem.motorHold();
    }
}
