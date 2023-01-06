package org.firstinspires.ftc.teamcode.FTCLib;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FTCLib.commands.MonkeCommands1;
import org.firstinspires.ftc.teamcode.FTCLib.commands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.RoadRunnerDooDoo.drive.SampleMecanumDrive;

@Autonomous(name = "Red_Right1", group = "FTCLib_Red")
public class Red_Right1 extends CommandOpMode {

    private MecanumDriveSubsystem peePee;
    private SimpleServo claw;
    @Override
    public void initialize() {
        peePee = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        // claw = new SimpleServo(MIN_ANGLE, MAX_ANGLE);
        schedule(new WaitUntilCommand(this:: isStarted), new MonkeCommands1(peePee));
    }
    //Servo claw = new SimpleServo(hardwareMap, "claw" , MIN_ANGLE, MAX_ANGLE);
}
