package frc.FRC6657.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;

public class RedHangarTwo extends SequentialCommandGroup{
    public RedHangarTwo (
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem pistons,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator
    ) {
        addCommands(
            new ParallelRaceGroup(
                new WaitUntilCommand(intake::ballDetected),
                drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_1, true)
            )
            .beforeStarting(
                new ParallelCommandGroup(
                    new InstantCommand(pistons::extend),
                    new InstantCommand(intake::start)
                )
            )
            .andThen(
                new ParallelCommandGroup(
                    new InstantCommand(pistons::retract),
                    new InstantCommand(intake::stop)
                )
            ),
            drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOOT, false)
            .beforeStarting(
                new InstantCommand(() -> flywheel.setRPMTarget(1000))
            )

            .andThen(
                new SequentialCommandGroup( //TODO This seems course, fix later.
                    new WaitUntilCommand(flywheel::atTarget),
                    new InstantCommand(accelerator::start),
                    new WaitCommand(0.5)
                )
            ),
            drivetrain.new TrajectoryFollowerCommand(PATH_TO_PARK, true)    
            );
    }

    private Trajectory PATH_TO_BALL_1 = Trajectories.generateTrajectory(3, 2, List.of(
        new Pose2d(10.4, 3.189, Rotation2d.fromDegrees(-35.815)),
        new Pose2d(11.515, 2.413, Rotation2d.fromDegrees(-37.705))

    ), 
    false, 
    "Red Mid 2 PATH_TO_BALL"
    );

    private Trajectory PATH_TO_SHOOT = Trajectories.generateTrajectory(1,2,List.of(
        new Pose2d(11.515, 2.413, Rotation2d.fromDegrees(-37.705)),
        new Pose2d(9.762, 3.404, Rotation2d.fromDegrees(-17.604))
    ),
    true,
    "Red Mid TWO PATH_TO_SHOOT"
    );

    private Trajectory PATH_TO_PARK = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(9.762, 3.404, Rotation2d.fromDegrees(-17.604)),
        new Pose2d(11.174, 2.207, Rotation2d.fromDegrees(-40.210))
    ), 
    false, 
    "Red Mid TWO PATH_TO_PARK"
    );
    
}

