
package org.firstinspires.ftc.teamcode.FTCLib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.FTCLib.commands.redmovement.redright.MonkeCommands1;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.RoadRunnerDooDoo.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.function.IntSupplier;

@Autonomous(name = "Red_Right1", group = "FTCLib_Red")
public class Red_Right1 extends CommandOpMode {
    private ElapsedTime time;

    // hardware declaration
    private SimpleServo clawServo;
    private Motor liftMotor;

    // subsystem declaration
    private MecanumDriveSubsystem driveSubsystem;
    private LiftSubsystem liftSubsystem;
    private ClawSubsystem clawSubsystem;

    @Override
    public void initialize() {
        // hardware initialization
        clawServo = new SimpleServo(hardwareMap, "claw", -90, 90);
        liftMotor = new Motor(hardwareMap, "slide");

        // subsystem initialization
        driveSubsystem = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        liftSubsystem = new LiftSubsystem(liftMotor);
        clawSubsystem = new ClawSubsystem(clawServo);

        //imageRec = new ImageRecognitionCommand(time);
        time = new ElapsedTime();

            schedule(new InstantCommand(() -> clawSubsystem.openClaw()));
            IntSupplier tagID = () -> 1;
            schedule(new WaitUntilCommand(this::isStarted)
                    .andThen(new InstantCommand(() -> clawSubsystem.closeClaw()))
                    .andThen(new MonkeCommands1(driveSubsystem, liftSubsystem, clawSubsystem, tagID)));
        }
    }




