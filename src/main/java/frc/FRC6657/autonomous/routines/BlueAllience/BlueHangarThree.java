package frc.FRC6657.autonomous.routines.BlueAllience;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.autonomous.common.AimRoutine;
import frc.FRC6657.autonomous.common.FireRoutine;
import frc.FRC6657.autonomous.common.IntakePath;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class BlueHangarThree extends SequentialCommandGroup{
    public BlueHangarThree (
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      ExtensionSubsystem pistons,
      FlywheelSubsystem flywheel,
      AcceleratorSubsystem accelerator,
      HoodSubsystem hood,
      VisionSupplier vision
    ) {
        addCommands(
          new InstantCommand(() -> drivetrain.resetPoseEstimator(PATH_TO_BALL_2.getInitialPose()), drivetrain), //Reset Position
          new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons), //Intake Blue 2
          new AimRoutine(drivetrain, hood, flywheel, vision), //Aim
          new FireRoutine(flywheel, hood, accelerator, 0.5), //Fire Blue 1 & 2
          new IntakePath(PATH_TO_BALL_3, drivetrain, intake, pistons), //Intkae Blue 3
          new AimRoutine(drivetrain, hood, flywheel, vision),
          new FireRoutine(flywheel, hood, accelerator, 0.5)
        );
    }

    private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(6, 4.68, Rotation2d.fromDegrees(91.87)),
        new Pose2d(5, 6.25, Rotation2d.fromDegrees(140))

    ), false, 
    "Blue Hangar 3 PATH_TO_BALL_2"
    );

    private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(4, 1, List.of(
      new Pose2d(5, 6.25, Rotation2d.fromDegrees(-60)),
      new Pose2d(5, 1.88, Rotation2d.fromDegrees(-140))

  ), false, 
  "Blue Hangar 3 PATH_TO_BALL_3"
  );
    
}
