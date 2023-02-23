package org.firstinspires.ftc.teamcode.FTCLib.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;
//subsystems
import org.firstinspires.ftc.teamcode.FTCLib.commands.LiftStop;
import org.firstinspires.ftc.teamcode.FTCLib.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.auto.CloseClawCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.auto.LiftCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.auto.LiftDownCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.auto.OpenClawCommand;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;

import java.util.HashMap;
import java.util.function.IntSupplier;

public class OrangutanCommandGroupRight extends SequentialCommandGroup {
    private final Pose2d startPos = new Pose2d(-63.0, -36.0, 0.0);
    public ElapsedTime timer = new ElapsedTime();

    public OrangutanCommandGroupRight(MecanumDriveSubsystem mecanumDriveSubsystem, LiftSubsystem liftSubsystem,
                                      ClawSubsystem clawSubsystem, IntSupplier tagID) {
        mecanumDriveSubsystem.setPoseEstimate(startPos);

        Trajectory traj1 = mecanumDriveSubsystem.trajectoryBuilder(startPos)
                .lineToSplineHeading(new Pose2d(-63.0, -12.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj2 = mecanumDriveSubsystem.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-24.0, 12.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj3 = mecanumDriveSubsystem.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-13.0, 12.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj4 = mecanumDriveSubsystem.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-12.0, 12.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj5 = mecanumDriveSubsystem.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(-13.0, 40.0, Math.toRadians(90.0)))
                .build();
        Trajectory traj6 = mecanumDriveSubsystem.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-12.0, 58.0, Math.toRadians(90.0)))
                .build();
        Trajectory traj7 = mecanumDriveSubsystem.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(-12.0, 24.0))
                .build();
        Trajectory traj8 = mecanumDriveSubsystem.trajectoryBuilder(traj7.end())
                .lineTo(new Vector2d(-13.0, 24.0))
                .build();
        // park positions start here

        addCommands(

        );

    }

}
