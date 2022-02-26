package frc.FRC6657.autonomous.routines.BlueAllience;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;

public class BlueBottomTarmacTwoBall extends SequentialCommandGroup{
    public BlueBottomTarmacTwoBall(
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem extension,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator
    ) {
        addCommands(
            new InstantCommand(extension::extend, extension),
            new ParallelRaceGroup(
                new WaitUntilCommand(intake::ballDetected),
                drivetrain.new TrajectoryFollowerCommand(backIntoBall, true)
            ),
            new InstantCommand(drivetrain::stop, drivetrain),
            new InstantCommand(() -> flywheel.setRPMTarget(1000)),
            new WaitUntilCommand(flywheel::atTarget),
            new InstantCommand(() -> accelerator.set(1))
        );
    }

    private Trajectory backIntoBall = Trajectories.generateTrajectory(1,1,List.of(
        new Pose2d(6.666, 2.737, Rotation2d.fromDegrees(0)),
        new Pose2d(5.164,2.044,Rotation2d.fromDegrees(0))
    ),
    true,
    "Back Into Ball"
    );
}
