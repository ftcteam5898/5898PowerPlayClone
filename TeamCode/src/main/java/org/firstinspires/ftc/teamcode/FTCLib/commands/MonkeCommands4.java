
package org.firstinspires.ftc.teamcode.FTCLib.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
// blueleft
public class MonkeCommands4 extends SequentialCommandGroup {
    private final Pose2d startPos = new Pose2d(63.0, -36.0, 180.0);

    public MonkeCommands4(MecanumDriveSubsystem mecanumDriveSubsystem) {
        mecanumDriveSubsystem.setPoseEstimate(startPos);
        Trajectory traj1 = mecanumDriveSubsystem.trajectoryBuilder(startPos)
                .forward(51.0)
                .build();
        Trajectory traj2 = mecanumDriveSubsystem.trajectoryBuilder(traj1.end())
                .strafeLeft(12.0)
                .build();
        // stack cone
        Trajectory traj3 = mecanumDriveSubsystem.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(13.0, -34.0))
                .build();
        Trajectory traj4 = mecanumDriveSubsystem.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(13.0, -36.0, Math.toRadians(270.0)))
                .build();
        Trajectory traj5 = mecanumDriveSubsystem.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(13.0, -55.0))
                .build();
        Trajectory traj6 = mecanumDriveSubsystem.trajectoryBuilder(traj5.end())
                .lineTo(new Vector2d(12.0, -58.0))
                .build();
        // pickup cone
        Trajectory traj7 = mecanumDriveSubsystem.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(13.0, -34.0))
                .build();
        Trajectory traj8 = mecanumDriveSubsystem.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(13.0, -36.0, Math.toRadians(180.0)))
                .build();
        Trajectory traj9 = mecanumDriveSubsystem.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(12.0, -24.0, Math.toRadians(180.0)))
                .build();
        // stack cone
        Trajectory traj10 = mecanumDriveSubsystem.trajectoryBuilder(traj9.end())
                .lineTo(new Vector2d(13.0, -34.0))
                .build();
        Trajectory traj11 = mecanumDriveSubsystem.trajectoryBuilder(traj10.end())
                .lineToSplineHeading(new Pose2d(13.0, -36.0, Math.toRadians(270.0)))
                .build();
        Trajectory traj12 = mecanumDriveSubsystem.trajectoryBuilder(traj11.end())
                .lineTo(new Vector2d(13.0, -55.0))
                .build();
        Trajectory traj13 = mecanumDriveSubsystem.trajectoryBuilder(traj12.end())
                .lineTo(new Vector2d(12.0, -58.0))
                .build();
        // pickup cone
        Trajectory traj14 = mecanumDriveSubsystem.trajectoryBuilder(traj13.end())
                .lineTo(new Vector2d(13.0, -34.0))
                .build();
        Trajectory traj15 = mecanumDriveSubsystem.trajectoryBuilder(traj14.end())
                .lineToSplineHeading(new Pose2d(13.0, -36.0, Math.toRadians(180.0)))
                .build();
        Trajectory traj16 = mecanumDriveSubsystem.trajectoryBuilder(traj15.end())
                .lineToSplineHeading(new Pose2d(12.0, -24.0, Math.toRadians(180.0)))
                .build();
        // stack cone


        //after trajectories
        addCommands(
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj1),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj2),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj3),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj4),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj5),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj6),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj7),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj8),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj9),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj10),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj11),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj12),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj13),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj14),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj15),
                new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj16)
        );
    }
}