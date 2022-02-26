package frc.FRC6657.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class RedTopTarmacTwoBall extends SequentialCommandGroup{
    public RedTopTarmacTwoBall(
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem extension,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator
    ) {
        addCommands(
            new InstantCommand(extension::extend, extension),
            new InstantCommand(() -> intake.set(1)),
            new ParallelRaceGroup(
                new WaitUntilCommand(intake::ballDetected),
                drivetrain.new TrajectoryFollowerCommand(backIntoBall, true)
            ),
            new InstantCommand(intake::stop),
            new InstantCommand(drivetrain::stop, drivetrain),
            new InstantCommand(() -> flywheel.setRPMTarget(1000)),
            new WaitUntilCommand(flywheel::atTarget),
            new InstantCommand(() -> accelerator.set(1)),
            new WaitCommand(.75),
            new InstantCommand(() -> accelerator.set(1)),
            new WaitCommand(.75),
            new InstantCommand(flywheel::stop),
            new InstantCommand(accelerator::stop)
        );
    }

    private Trajectory backIntoBall = Trajectories.generateTrajectory(1,1,List.of(
        new Pose2d(9.802, 5.55, Rotation2d.fromDegrees(210)),
        new Pose2d(11.218, 6.353,Rotation2d.fromDegrees(210))
    ),
    true,
    "Back Into Ball"
    );
}
