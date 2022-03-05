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

public class BlueDoubleSteal extends SequentialCommandGroup{
    public BlueDoubleSteal(
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
                new SequentialCommandGroup( //TODO This seems coarse, fix later.
                    new WaitUntilCommand(flywheel::atTarget),
                    new InstantCommand(accelerator::start),
                    new WaitCommand(0.5)
                )
            ),
            new ParallelRaceGroup(
                new WaitUntilCommand(intake::ballDetected),
                drivetrain.new TrajectoryFollowerCommand(PATH_TO_RED_1, true)
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
            drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOOT_RED, false)
            .beforeStarting(
                new InstantCommand(() -> flywheel.setRPMTarget(1000))
            )
            .andThen(
                new SequentialCommandGroup( //TODO This seems coarse, fix later.
                    new WaitUntilCommand(flywheel::atTarget),
                    new InstantCommand(accelerator::start),
                    new WaitCommand(0.5)
                )
            )
         //   drivetrain.new TrajectoryFollowerCommand(PATH_TO_PARK, true)    
        );
    }

    private Trajectory PATH_TO_BALL_1 = Trajectories.generateTrajectory(3, 2, List.of(
        new Pose2d(6.140, 5.179, Rotation2d.fromDegrees(-220)),
        new Pose2d(5.014, 6.16, Rotation2d.fromDegrees(-220))

    ), false, 
    "Blue Steal 2 PATH_TO_BALL"
    );

    private Trajectory PATH_TO_SHOOT = Trajectories.generateTrajectory(1,2,List.of(
        new Pose2d(5.014, 6.16, Rotation2d.fromDegrees(-220)),
        new Pose2d(6.766, 4.661, Rotation2d.fromDegrees(155.98))
    ),
    true,
    "Blue Steal PATH_TO_SHOOT"
    );

    private Trajectory PATH_TO_RED_1 = Trajectories.generateTrajectory(1, 2,List.of(
        new Pose2d(6.766, 4.661, Rotation2d.fromDegrees(155.98)),
        new Pose2d(6.122, 7.082, Rotation2d.fromDegrees(96))

    ), 
    false, 
    "Blue Steal PATH_TO_RED_1"
    );

    private Trajectory PATH_TO_SHOOT_RED = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(6.122, 7.082, Rotation2d.fromDegrees(96)),
        new Pose2d(6.122, 7.082, Rotation2d.fromDegrees(12))
    ), 
    true, 
    "Blue Steal PATH_TO_SHOOT_RED"
    );

    private Trajectory PATH_TO_PARK = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(6.766, 4.661, Rotation2d.fromDegrees(155.98)),
        new Pose2d(5.014, 6.16, Rotation2d.fromDegrees(-223))
    ), 
    false, 
    "Blue Steal TWO PATH_TO_PARK"
    );


    }
