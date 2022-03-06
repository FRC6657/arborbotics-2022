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

public class RedMidTwo extends SequentialCommandGroup{
    public RedMidTwo(
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

            //TODO Retract Intake
            new InstantCommand(intake::stop),
            
            new InstantCommand(drivetrain::stop, drivetrain),

            //TODO Set target when driving to ball 2
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
        new Pose2d(9.802, 5.55, Rotation2d.fromDegrees(210)),
        new Pose2d(11.218, 6.353,Rotation2d.fromDegrees(210))
    ),
    true,
    "RedMidTwo PATH_TO_BALL_2"
    );
}
