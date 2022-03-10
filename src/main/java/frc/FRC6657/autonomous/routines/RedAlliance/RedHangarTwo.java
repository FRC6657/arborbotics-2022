package frc.FRC6657.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.custom.ArborSequentialCommandGroup;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class RedHangarTwo extends ArborSequentialCommandGroup{
    public RedHangarTwo (
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem pistons,
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        AcceleratorSubsystem accelerator,
        VisionSupplier vision
    ) {
        addReqs(drivetrain, intake, pistons, flywheel, hood, accelerator, vision);
        addCommands(
            drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_1, true)
            .beforeStarting(
                new ParallelCommandGroup( //Prepare Intake to pickup ball #2
                  new InstantCommand(pistons::extend), 
                  new InstantCommand(intake::start)
                )
              )
              .andThen(
                new ParallelCommandGroup( //Retract Intake After Ball #2
                  new InstantCommand(pistons::retract),
                  new InstantCommand(intake::stop)
                )
              ),
            new TurnAndShoot()
        );
    }

    private Trajectory PATH_TO_BALL_1 = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(10.5, 3.55, Rotation2d.fromDegrees(-89.15)),
        new Pose2d(11.58, 2, Rotation2d.fromDegrees(-30))

    ), false, 
    "Red Hangar 2 PATH_TO_BALL"
    );
    
}
