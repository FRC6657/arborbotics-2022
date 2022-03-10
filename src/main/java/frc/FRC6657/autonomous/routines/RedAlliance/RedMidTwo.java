package frc.FRC6657.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.custom.ArborSequentialCommandGroup;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class RedMidTwo extends ArborSequentialCommandGroup{
    public RedMidTwo(
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem extension,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator,
        HoodSubsystem hood,
        VisionSupplier vision
    ) {
        addReqs(drivetrain, intake, extension, flywheel, hood, accelerator, vision);
        addCommands(

            //TODO Make this one thing and use a constant
            new InstantCommand(extension::extend, extension),
            new InstantCommand(intake::start, intake),

            new ParallelRaceGroup(
                new WaitUntilCommand(intake::ballDetected),
                drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_2, true)
            ),

            //TODO Retract Intake
            new InstantCommand(intake::stop, intake),
            
            new InstantCommand(drivetrain::stop, drivetrain),

            //Start shooting
           
      new TurnAndShoot()
      );
    }

    private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(1,1,List.of(
        new Pose2d(9.802, 5.55, Rotation2d.fromDegrees(24)),
        new Pose2d(11.218, 6.353,Rotation2d.fromDegrees(24))
    ),
    false,
    "RedMidTwo PATH_TO_BALL_2"
    );
}
