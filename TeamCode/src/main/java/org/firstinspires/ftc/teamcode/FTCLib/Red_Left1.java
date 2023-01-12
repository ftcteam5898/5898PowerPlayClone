
package org.firstinspires.ftc.teamcode.FTCLib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FTCLib.commands.redmovement.redleft.MonkeCommands3;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.RoadRunnerDooDoo.drive.SampleMecanumDrive;

@Autonomous(name = "Red_Left1", group = "FTCLib_Red")
public class Red_Left1 extends CommandOpMode {

    // hardware declarations
    private SimpleServo clawServo;
    private Motor liftMotor;

    // subsystem declarations
    private MecanumDriveSubsystem driveSubsystem;
    private LiftSubsystem liftSubsystem;
    private ClawSubsystem clawSubsystem;

    @Override
    public void initialize() {
        // hardware initializations
        clawServo = new SimpleServo(hardwareMap, "claw", -90, 90);
        liftMotor = new Motor(hardwareMap, "slide");

        driveSubsystem = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        liftSubsystem = new LiftSubsystem(liftMotor);
        clawSubsystem = new ClawSubsystem(clawServo);

        // opens claw upon initialization
        new InstantCommand(() -> clawSubsystem.openClaw());

        schedule(new WaitUntilCommand(this:: isStarted)
                .andThen(new InstantCommand(() -> clawSubsystem.closeClaw()))
                .andThen(new MonkeCommands3(driveSubsystem)));
    }
    //Servo claw = new SimpleServo(hardwareMap, "claw" , MIN_ANGLE, MAX_ANGLE);
}

