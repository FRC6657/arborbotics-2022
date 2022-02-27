package frc.FRC6657.autonomous.routines.BlueAllience;

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

public class BlueMidTwo extends SequentialCommandGroup{
    public BlueMidTwo(
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem extension,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator
    ) {
        addCommands(

            //TODO Make this one thing and use a constant
            new InstantCommand(extension::extend, extension),
            new InstantCommand(() -> intake.set(1)),

            new ParallelRaceGroup(
                new WaitUntilCommand(intake::ballDetected),
                drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_2, true)
            ),
            new InstantCommand(intake::stop),
            new InstantCommand(drivetrain::stop, drivetrain),
            new InstantCommand(() -> flywheel.setRPMTarget(1000)),
            new WaitUntilCommand(flywheel::atTarget),
            new InstantCommand(() -> accelerator.set(1)),
            new WaitCommand(.75), //TODO Make this not timed
            new InstantCommand(() -> accelerator.set(1)),
            new WaitCommand(.75), //TODO Make this not timed
            new InstantCommand(flywheel::stop),
            new InstantCommand(accelerator::stop)
        );
    }

    private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(1,1,List.of(
        new Pose2d(6.666, 2.737, Rotation2d.fromDegrees(200)),
        new Pose2d(5.164,2.044,Rotation2d.fromDegrees(200))
    ),
    false,
    "BlueMidTwo PATH_TO_BALL_2"
    );
}
