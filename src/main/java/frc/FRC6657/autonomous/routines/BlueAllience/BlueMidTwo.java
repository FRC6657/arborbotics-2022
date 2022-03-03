package frc.FRC6657.autonomous.routines.BlueAllience;

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

public class BlueMidTwo extends SequentialCommandGroup{
    public BlueMidTwo(
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem pistons,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator
    ) {
        addCommands(
            new ParallelCommandGroup(
                new InstantCommand(pistons::extend),
                new InstantCommand(intake::start),
                new ParallelRaceGroup(
                    new WaitUntilCommand(intake::ballDetected),
                    drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL, true)
                )
            ).andThen(
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
                new SequentialCommandGroup(
                    new InstantCommand(drivetrain::stop),
                    new WaitUntilCommand(flywheel::atTarget),
                    new InstantCommand(accelerator::start),
                    new WaitCommand(0.5)
                )
            ),
            drivetrain.new TrajectoryFollowerCommand(PATH_TO_EXIT, false)
            .beforeStarting(
                new ParallelCommandGroup(
                    new InstantCommand(accelerator::stop),
                    new InstantCommand(flywheel::stop)
                )
            )
        );
    }

    private Trajectory PATH_TO_BALL = Trajectories.generateTrajectory(3,2,List.of(
        new Pose2d(7.722, 2.105, Rotation2d.fromDegrees(181.795)),
        new Pose2d(5.263, 2.092, Rotation2d.fromDegrees(181.795))
    ),
    false,
    "Blue Mid TWO PATH_TO_BALL"
    );

    private Trajectory PATH_TO_SHOOT = Trajectories.generateTrajectory(1,2,List.of(
        new Pose2d(5.263, 2.092, Rotation2d.fromDegrees(181.795)),
        new Pose2d(7.471, 2.317, Rotation2d.fromDegrees(235.681))
    ),
    true,
    "Blue Mid TWO PATH_TO_SHOOT"
    );

    private Trajectory PATH_TO_EXIT = Trajectories.generateTrajectory(2,1,List.of(
        new Pose2d(7.471, 2.317, Rotation2d.fromDegrees(235.681)),
        new Pose2d(4.75, 3.33, Rotation2d.fromDegrees(150))
    ),
    false,
    "Blue Mid TWO PATH_TO_EXIT"
    );

}
