package org.firstinspires.ftc.teamcode.FTCLib.commands;

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
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.RoadRunnerDooDoo.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunnerDooDoo.drive.SampleMecanumDrive;

import java.util.HashMap;
import java.util.function.IntSupplier;

public class GorillaCommandGroupLeft extends SequentialCommandGroup {
    private final Pose2d startPos = new Pose2d(-63.0, 36.0, 0.0);
    public ElapsedTime timer = new ElapsedTime();
    private double timeToLift;

    public GorillaCommandGroupLeft(MecanumDriveSubsystem mecanumDriveSubsystem, LiftSubsystem liftSubsystem,
                                   ClawSubsystem clawSubsystem, IntSupplier tagID) {
        mecanumDriveSubsystem.setPoseEstimate(startPos);

        Trajectory traj1 = mecanumDriveSubsystem.trajectoryBuilder(startPos)
                .lineToSplineHeading(new Pose2d(-63.0, 12.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj2 = mecanumDriveSubsystem.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-35.0, 11.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj3 = mecanumDriveSubsystem.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-34.0, 1.0))
                .build();
        Trajectory traj4 = mecanumDriveSubsystem.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-35.0, 12.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj5 = mecanumDriveSubsystem.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(-13.0, 12.0))
                .build();
        Trajectory traj6 = mecanumDriveSubsystem.trajectoryBuilder(traj5.end())
                .lineToSplineHeading(new Pose2d(-12.0, 12.0, Math.toRadians(90.0)))
                .build();
        Trajectory trajpain = mecanumDriveSubsystem.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(-13.0, 50.0, Math.toRadians(90.0)))
                .build();
        Trajectory traj7 = mecanumDriveSubsystem.trajectoryBuilder(trajpain.end())
                .lineToSplineHeading(new Pose2d(-12.0, 58.7, Math.toRadians(90.0)))
                .build();
        Trajectory traj8 = mecanumDriveSubsystem.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(-13.0, 34.0, Math.toRadians(90.0)))
                .build();
        Trajectory traj9 = mecanumDriveSubsystem.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(-12.0, 36.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj10 = mecanumDriveSubsystem.trajectoryBuilder(traj9.end())
                .lineToSplineHeading(new Pose2d(-8.5, 21.5, Math.toRadians(0.0)))
                .build();
        Trajectory traj11 = mecanumDriveSubsystem.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(-11.0, 21.5, Math.toRadians(0.0)))
                .build();
        // park position code
        // third position
        Trajectory traj12 = mecanumDriveSubsystem.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(-11.0, 11.0, Math.toRadians(90.0)))
                .build();
        // second position
        Trajectory traj13 = mecanumDriveSubsystem.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(-13.0, 34.0, Math.toRadians(0.0)))
                .build();
        // first position
        Trajectory traj14 = mecanumDriveSubsystem.trajectoryBuilder(traj11.end())
                .lineToSplineHeading(new Pose2d(-11.0, 58.5, Math.toRadians(0.0)))
                .build();

        addCommands(
                new WaitCommand(600),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj1),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj2).alongWith(
                new LiftCommand(liftSubsystem, timer, 3.2)),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj3),
                new WaitCommand(1000),
                new WaitCommand(500).deadlineWith(new OpenClawCommand(clawSubsystem)),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj4),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj5).alongWith(
                        new LiftDownCommand(liftSubsystem, timer)),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj6),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajpain),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj7).alongWith(
                        new LiftCommand(liftSubsystem, timer, 0.4)),
                new WaitCommand(500).deadlineWith(new CloseClawCommand(clawSubsystem)),
                new LiftCommand(liftSubsystem, timer, 0.5),
                new LiftCommand(liftSubsystem, timer, 0.4),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj8),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj9).alongWith(
                        new LiftCommand(liftSubsystem, timer, 2.5)),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj10),
                new WaitCommand(1000),
                new WaitCommand(500).deadlineWith(new OpenClawCommand(clawSubsystem)),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj11),
                new LiftDownCommand(liftSubsystem, timer),
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(3, new ParallelCommandGroup(new TrajectoryFollowerCommand(mecanumDriveSubsystem,
                            traj12), new WaitCommand(500).deadlineWith(new LiftStop(liftSubsystem)),
                            new WaitCommand(500).deadlineWith(new CloseClawCommand(clawSubsystem))));
                    put(2, new ParallelCommandGroup(new TrajectoryFollowerCommand(mecanumDriveSubsystem,
                            traj13), new WaitCommand(500).deadlineWith(new LiftStop(liftSubsystem))));
                    put(1, new ParallelCommandGroup(new TrajectoryFollowerCommand(mecanumDriveSubsystem,
                            traj14), new WaitCommand(500).deadlineWith(new LiftStop(liftSubsystem)),
                            new WaitCommand(500).deadlineWith(new CloseClawCommand(clawSubsystem))));
                }}, () -> tagID.getAsInt() // selector yippee im in pain
            )

        );

    }
}
