
package org.firstinspires.ftc.teamcode.FTCLib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTCLib.commands.bluemovement.blueleft.MonkeCommands4;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.RoadRunnerDooDoo.drive.SampleMecanumDrive;

@Autonomous(name = "Blue_Left1", group = "FTCLib_Blue")
public class Blue_Left1 extends CommandOpMode {

    private MecanumDriveSubsystem driveSubsystem;
    private SimpleServo claw;
    @Override
    public void initialize() {
        driveSubsystem = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        // claw = new SimpleServo(MIN_ANGLE, MAX_ANGLE);
        schedule(new WaitUntilCommand(this:: isStarted), new MonkeCommands4(driveSubsystem));
    }
    //Servo claw = new SimpleServo(hardwareMap, "claw" , MIN_ANGLE, MAX_ANGLE);
}

