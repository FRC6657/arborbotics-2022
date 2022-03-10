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

public class BlueWallThree extends SequentialCommandGroup {
  /** Creates a new BlueFive. */
  public BlueWallThree(
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
      new IntakePath(PATH_TO_BALL_3, drivetrain, intake, pistons),
      new AimRoutine(drivetrain, hood, flywheel, vision),
      new FireRoutine(flywheel, hood, accelerator, 0.5)
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3,6,List.of(
    new Pose2d(7.821, 1.922, Rotation2d.fromDegrees(-89.018)),
    new Pose2d(7.784, 1, Rotation2d.fromDegrees(-89.018))
  ),
  false,
  "Blue Wall Three TWO PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(3,4,List.of(
    new Pose2d(7.885, 2.629, Rotation2d.fromDegrees(-150)),
    new Pose2d(5.022, 1.75, Rotation2d.fromDegrees(-180))
  ),
  false,
  "Blue Wall Three TWO PATH_TO_BALL_3"
  );

}

