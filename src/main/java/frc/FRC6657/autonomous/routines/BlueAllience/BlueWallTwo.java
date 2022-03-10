// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
public class BlueWallTwo extends SequentialCommandGroup {
  public BlueWallTwo(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake,
    ExtensionSubsystem pistons,
    FlywheelSubsystem flywheel,
    AcceleratorSubsystem accelerator,
    HoodSubsystem hood,
    VisionSupplier vision
  ) {
    addCommands(
      new InstantCommand(() -> drivetrain.resetPoseEstimator(PATH_TO_BALL_2.getInitialPose()), drivetrain),
      new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons),
      new AimRoutine(drivetrain, hood, flywheel, vision),
      new FireRoutine(flywheel, hood, accelerator, 0.5),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_TAXI)
    );
  }
  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(1, 2, List.of(
    new Pose2d(7.625, 1.95, Rotation2d.fromDegrees(-87.616)),
    new Pose2d(7.6125, 1, Rotation2d.fromDegrees(-90))
  ), false, 
  "Blue Wall 2 PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_TAXI = Trajectories.generateTrajectory(1, 2, List.of(
    new Pose2d(7.6125, 0.79, Rotation2d.fromDegrees(0)),
    new Pose2d(5.5, 0.65, Rotation2d.fromDegrees(0))
  ), true, 
  "Blue Wall 2 PATH_TO_TAXI"
  );

}
