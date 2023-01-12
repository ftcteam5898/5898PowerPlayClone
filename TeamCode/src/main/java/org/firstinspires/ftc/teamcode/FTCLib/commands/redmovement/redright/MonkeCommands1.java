
package org.firstinspires.ftc.teamcode.FTCLib.commands.redmovement.redright;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;
//subsystems
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.FTCLib.Red_Right1;
import org.firstinspires.ftc.teamcode.FTCLib.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.LiftDownCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.LiftStop;
import org.firstinspires.ftc.teamcode.FTCLib.commands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
// image recognition stuff
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class MonkeCommands1 extends SequentialCommandGroup
{
    private final Pose2d startPos = new Pose2d(-63.0, -36.0, 0.0);
    private ElapsedTime timer = new ElapsedTime();
    private double timeToLift;

    // a lot of pain
    private Red_Right1 pain;

    public String ligma;
    public Red_Right1 a;

    // to be used later
    private Motor liftMotor;
    private LiftSubsystem lift;

    public MonkeCommands1(MecanumDriveSubsystem mecanumDriveSubsystem,
                          LiftSubsystem liftSubsystem, ClawSubsystem clawSubsystem,
                          String deez)
    {
        a.setLabel(ligma);
        deez = ligma;

        LiftSubsystem lift = new LiftSubsystem(liftMotor);


        mecanumDriveSubsystem.setPoseEstimate(startPos);
        Trajectory traj1 = mecanumDriveSubsystem.trajectoryBuilder(startPos)
                .lineToSplineHeading(new Pose2d(-13.0, -36.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj2 = mecanumDriveSubsystem.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-12.0, -24.0, Math.toRadians(0.0)))
                .build();
        // stack cone
        // temporary park code remove later
        Trajectory trajdeez = mecanumDriveSubsystem.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-13.0, -24.0))
                .build();
        Trajectory trajy = mecanumDriveSubsystem.trajectoryBuilder(trajdeez.end())
                .lineTo(new Vector2d(-13.0, -13.0))
                .build();
        // end of temporary code
        Trajectory traj3 = mecanumDriveSubsystem.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-13.0, -34.0))
                .build();
        Trajectory traj4 = mecanumDriveSubsystem.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-13.0, -36.0, Math.toRadians(270.0)))
                .build();
        Trajectory traj5 = mecanumDriveSubsystem.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(-13.0, -55.0))
                .build();
        Trajectory traj6 = mecanumDriveSubsystem.trajectoryBuilder(traj5.end())
                .lineTo(new Vector2d(-12.0, -58.0))
                .build();
        // pickup cone
        Trajectory traj7 = mecanumDriveSubsystem.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(-13.0, -34.0))
                        .build();
        Trajectory traj8 = mecanumDriveSubsystem.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(-13.0, -36.0, Math.toRadians(0.0)))
                        .build();
        Trajectory traj9 = mecanumDriveSubsystem.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(-12.0, -24.0, Math.toRadians(0.0)))
                        .build();
        // stack cone
        Trajectory traj10 = mecanumDriveSubsystem.trajectoryBuilder(traj9.end())
                .lineTo(new Vector2d(-13.0, -34.0))
                        .build();
        Trajectory traj11 = mecanumDriveSubsystem.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(-13.0, -36.0, Math.toRadians(270.0)))
                        .build();
        Trajectory traj12 = mecanumDriveSubsystem.trajectoryBuilder(traj11.end())
                .lineTo(new Vector2d(-13.0, -55.0))
                        .build();
        Trajectory traj13 = mecanumDriveSubsystem.trajectoryBuilder(traj12.end())
                .lineTo(new Vector2d(-12.0, -58.0))
                        .build();
        // pickup cone
        Trajectory traj14 = mecanumDriveSubsystem.trajectoryBuilder(traj13.end())
                .lineTo(new Vector2d(-13.0, -34.0))
                        .build();
        Trajectory traj15 = mecanumDriveSubsystem.trajectoryBuilder(traj14.end())
                .lineToSplineHeading(new Pose2d(-13.0, -36.0, Math.toRadians(0)))
                        .build();
        Trajectory traj16 = mecanumDriveSubsystem.trajectoryBuilder(traj15.end())
                .lineToSplineHeading(new Pose2d(-12.0, -24.0, Math.toRadians(0.0)))
                        .build();
        // stack cone


        //after trajectories
        addCommands(
                new PrintCommand(deez),
                new WaitCommand(600),
                // allowing the slide to move while the first command rush
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj1).alongWith(
                        new LiftCommand(liftSubsystem, timer)),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj2),
                // drop cone and wait till its done before continuing the trajectories
                new WaitCommand(1500),
                new WaitCommand(500).deadlineWith(new OpenClawCommand(clawSubsystem))
                // temporary park code remove later
                //new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajdeez),
                //new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajy).alongWith(
                        //new LiftStop(liftSubsystem))
                /* join later
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj3),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj4).alongWith(
                        new LiftDownCommand(liftSubsystem, timer))
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj5),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj6),
                // pickup cone
                new WaitCommand(500).deadLineWith(new CloseClawCommand(clawSubsystem)),

                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj7),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj8),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj9).alongWith(
                        new LiftCommand(liftSubsystem, timer)),
                // drop cone and wait till its done before continuing the trajectories
                new WaitCommand(500).deadLineWith(new OpenClawCommand(clawSubsystem)),

                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj10).alongWith(
                        new LiftDownCommand(liftSubsystem, timer)),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj11),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj12),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj13),
                // pickup cone
                new WaitCommand(500).deadLineWith(new CloseClawCommand(clawSubsystem)),

                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj14),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj15).alongWith(
                        new LiftCommand(liftSubsystem, timer)),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj16),
                new WaitCommand(500).deadLineWith(new OpenClawCommand(clawSubsystem))
                // stack
                 */
        );

        if (deez.equals("purpleDuck")) {
            addCommands(
                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajdeez),
                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajy).alongWith(
                            new LiftStop(liftSubsystem))
            );
        }
        else if (deez.equals("mallard")) {
            addCommands(
                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajdeez)
            );
        }

    }
}

