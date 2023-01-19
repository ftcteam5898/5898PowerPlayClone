
package org.firstinspires.ftc.teamcode.FTCLib.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;
//subsystems
import org.firstinspires.ftc.teamcode.FTCLib.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.LiftStop;
import org.firstinspires.ftc.teamcode.FTCLib.commands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.FTCLib.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTCLib.subsystems.MecanumDriveSubsystem;
// image recognition stuff

import java.util.HashMap;
import java.util.function.IntSupplier;

public class MonkeCommandsRight extends SequentialCommandGroup {
    private final Pose2d startPos = new Pose2d(-63.0, -36.0, 0.0);
    public ElapsedTime timer = new ElapsedTime();
    private double timeToLift;

    // to be used later
    private Motor liftMotor;
    private LiftSubsystem lift;

    public MonkeCommandsRight(MecanumDriveSubsystem mecanumDriveSubsystem,
                              LiftSubsystem liftSubsystem, ClawSubsystem clawSubsystem, IntSupplier tagID) {
        {
            LiftSubsystem lift = new LiftSubsystem(liftMotor);

            mecanumDriveSubsystem.setPoseEstimate(startPos);
            Trajectory traj1 = mecanumDriveSubsystem.trajectoryBuilder(startPos)
                    .lineToSplineHeading(new Pose2d(-11.0, -36.0, Math.toRadians(0.0)))
                    .build();
            Trajectory traj2 = mecanumDriveSubsystem.trajectoryBuilder(traj1.end())
                    .lineToSplineHeading(new Pose2d(-11.0, -25.5, Math.toRadians(0.0)))
                    .build();
            // stack cone
            // temporary park code remove later
            Trajectory trajdeez = mecanumDriveSubsystem.trajectoryBuilder(traj2.end())
                    .lineTo(new Vector2d(-13.0, -25.0))
                    .build();
            // first park position code
            Trajectory trajy = mecanumDriveSubsystem.trajectoryBuilder(trajdeez.end())
                    .lineToSplineHeading(new Pose2d(-13.0, -13.0, Math.toRadians(270.0)))
                    .build();
            // second park position code
            Trajectory traj = mecanumDriveSubsystem.trajectoryBuilder(trajdeez.end())
                    .lineToSplineHeading(new Pose2d(-13.0, -36.0, Math.toRadians(270.0)))
                    .build();
            // third park position code
            Trajectory trajp = mecanumDriveSubsystem.trajectoryBuilder(trajdeez.end())
                    .lineToSplineHeading(new Pose2d(-13.0, -58.5, Math.toRadians(0.0)))
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
                    new WaitCommand(600),
                    // allowing the slide to move while the first command rush
                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj1).alongWith(
                            new LiftCommand(liftSubsystem, timer, 3.4)),
                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj2),
                    // drop cone and wait till its done before continuing the trajectories
                    new WaitCommand(1500),
                    new WaitCommand(500).deadlineWith(new OpenClawCommand(clawSubsystem)),
                    // temporary park code positions selector go pee pee poo poo
                    new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajdeez),
                    new SelectCommand(new HashMap<Object, Command>() {{
                        put(1, new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajy)
                                .alongWith(new LiftStop(liftSubsystem)));
                        put(2, new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj).alongWith(
                                new LiftStop(liftSubsystem)));
                        put(3, new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajp).alongWith(
                                new LiftStop(liftSubsystem)));
                    }}, () -> tagID.getAsInt() // selector yippee im in pain
                    )
                    //first park position code
                    //new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajy).alongWith(
                    //      new LiftStop(liftSubsystem))
                    // second park position code
                    //new TrajectoryFollowerCommand(mecanumDriveSubsystem, traj).alongWith(
                            //new LiftStop(liftSubsystem)),
                    // third park position code
                    //new TrajectoryFollowerCommand(mecanumDriveSubsystem, trajp).alongWith(
                    //new LiftStop(liftSubsystem)),
                    //new WaitCommand(500).deadlineWith(new CloseClawCommand(clawSubsystem)) // apart of all park positions
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

        }


    }
}