// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines.BlueAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

public class BlueThreeWall extends ArborSequentialCommandGroup {
  /** Creates a new BlueFive. */
  public BlueThreeWall(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake,
    ExtensionSubsystem pistons,
    FlywheelSubsystem flywheel,
    AcceleratorSubsystem accelerator,
    HoodSubsystem hood,
    VisionSupplier vision
  ) {
    addReqs(drivetrain, intake, pistons, flywheel, hood, accelerator, vision);
    addCommands(
      new ParallelRaceGroup(
        new WaitUntilCommand(intake::ballDetected),
        drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_2, true)
      ).beforeStarting(
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
      new TurnAndShoot(drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOT_1, false)),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_3, false)
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
      new TurnAndShoot(drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOT_2, false)),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_EXIT, false)
      );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3,6,List.of(
    new Pose2d(7.821, 1.922, Rotation2d.fromDegrees(-89.018)),
    new Pose2d(7.784, 0.623, Rotation2d.fromDegrees(-89.018))
  ),
  false,
  "Blue Five TWO PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_SHOT_1 = Trajectories.generateTrajectory(1.5,4,List.of(
    new Pose2d(7.784, 0.623, Rotation2d.fromDegrees(-89.018)),
    new Pose2d(7.885, 2.629, Rotation2d.fromDegrees(-108.334))
  ),
  true,
  "Blue Five TWO PATH_TO_SHOT_1"
  );

  private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(3,4,List.of(
    new Pose2d(7.885, 2.629, Rotation2d.fromDegrees(-108.334)),
    new Pose2d(5.022, 2.024, Rotation2d.fromDegrees(-158.408))
  ),
  false,
  "Blue Five TWO PATH_TO_BALL_3"
  );

  private Trajectory PATH_TO_SHOT_2 = Trajectories.generateTrajectory(1.5,4,List.of(
    new Pose2d(5.022, 2.024, Rotation2d.fromDegrees(-158.408)),
    new Pose2d(7.885, 2.629,Rotation2d.fromDegrees(-128.334))
  ),
  true,
  "Blue Five TWO PATH_TO_SHOT_2"
  );

  private Trajectory PATH_TO_EXIT = Trajectories.generateTrajectory(4,4,List.of(
    new Pose2d(7.885, 2.4, Rotation2d.fromDegrees(-122.334)),
    new Pose2d(6.55, 1.4, Rotation2d.fromDegrees(-133.334))
  ),
  false,
  "Blue Five TWO PATH_TO_EXIT"
  );

}


