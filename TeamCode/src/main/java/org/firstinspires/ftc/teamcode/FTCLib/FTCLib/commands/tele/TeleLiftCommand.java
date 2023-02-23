package org.firstinspires.ftc.teamcode.FTCLib.commands.tele;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftEncoderSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;

public class TeleLiftCommand extends CommandBase {

    private LiftSubsystem liftSubsystem;
    private LiftEncoderSubsystem encoderSubsystem;
    private ElapsedTime time;
    private int junction = 0; // 0 = low, 1 = medium, 2 = high
    private double liftTime = 0.4;
    private Motor liftMotor;

    public TeleLiftCommand(LiftEncoderSubsystem subsystem) {
        encoderSubsystem = subsystem;
        liftMotor = encoderSubsystem.getLiftMotor();
    }

    public TeleLiftCommand(LiftSubsystem subsystem, ElapsedTime timer) {
        liftSubsystem = subsystem;
        time = timer;
    }
    @Override
    public void initialize() {
        encoderSubsystem.motorUp();
    }
    @Override
    public boolean isFinished() {
        // change values to correct junction position
        if (junction == 0) {
            return liftMotor.getCurrentPosition() <= -3000;
        }
        else if (junction == 1) {
            return liftMotor.getCurrentPosition() <= -3000;
        }
        else if (junction == 2) {
            return liftMotor.getCurrentPosition() <= -3000;
        }
        else {
            return liftMotor.getCurrentPosition() <= -3000;
        }
    }
    @Override
    public void end(boolean interupted) {
        if (junction == 0) {
            junction ++;
            encoderSubsystem.motorHold();
        }
        else if (junction == 1) {
            junction ++;
            encoderSubsystem.motorHold();
        }
        else if(junction == 2) {
            junction++;
            encoderSubsystem.motorHold();
        }
        else {
            junction = 0;
            encoderSubsystem.allTheWayDown();
        }
    }

    /*
    @Override
    public void initialize() {
        time.reset();
        if (junction == 0) {
            liftSubsystem.motorUp();
            liftSubsystem.motorHold();
            liftTime = 0.5;
            junction++;
        }
        else if (junction == 1) {
            liftSubsystem.motorUp();
            liftTime = 0.9;
            junction++;
        }
        else if (junction == 2) {
            liftSubsystem.motorUp();
            liftTime = 1.4;
            junction++;
        }
        else {
            liftSubsystem.motorDown();
            liftTime = 2.0;
            junction = 0;
        }
    }
    @Override
    public boolean isFinished() {
        return time.seconds() >= liftTime;
    }
    @Override
    public void end(boolean interupted) {
        if (junction != 3) {
            liftSubsystem.motorHold();
        }
        else {
            liftSubsystem.motorStop();
        }
    }
     */
}