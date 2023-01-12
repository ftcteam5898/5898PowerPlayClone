
package org.firstinspires.ftc.teamcode.FTCLib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTCLib.commands.bluemovement.blueright.MonkeCommands2;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.RoadRunnerDooDoo.drive.SampleMecanumDrive;

@Autonomous(name = "Blue_Right1", group = "FTCLib_Blue")
public class Blue_Right1 extends CommandOpMode {
    // hardware declarations
    private SimpleServo claw;
    private Motor lift;

    private MecanumDriveSubsystem driveSubsystem;
    private LiftSubsystem liftSubsystem;
    private ClawSubsystem clawSubsystem;

    @Override
    public void initialize() {
        driveSubsystem = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        // claw = new SimpleServo(MIN_ANGLE, MAX_ANGLE);

        schedule(new WaitUntilCommand(this:: isStarted), new MonkeCommands2(driveSubsystem,
                liftSubsystem, clawSubsystem));
    }
    //Servo claw = new SimpleServo(hardwareMap, "claw" , MIN_ANGLE, MAX_ANGLE);
}

