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
import frc.FRC6657.autonomous.common.ShootPath;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class BlueFive extends SequentialCommandGroup{
  /** Creates a new BlueFive. */
  public BlueFive(
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
      new AimRoutine(drivetrain, hood, flywheel, vision).withTimeout(1.5), //Aim
      new FireRoutine(flywheel, hood, accelerator, 0.5).withTimeout(1.5), //Fire Blue 1 & 2
      new IntakePath(PATH_TO_BALL_3, drivetrain, intake, pistons), // Intake Blue 3
      new AimRoutine(drivetrain, hood, flywheel, vision).withTimeout(1.5), //Aim 
      new FireRoutine(flywheel, hood, accelerator, 0.5).withTimeout(1.5), //Fire Blue 3
      new IntakePath(PATH_TO_BALL_4_5, drivetrain, intake, pistons), //Intake Blue 4 & 5
      new ShootPath(PATH_TO_4_5_SHOT, drivetrain, hood, flywheel, vision), //Move to firing position
      new FireRoutine(flywheel, hood, accelerator, 0.5).withTimeout(1.5) //Fire Blue 4 & 5
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3,6,List.of(
    new Pose2d(7.821, 1.922, Rotation2d.fromDegrees(-89.018)),
    new Pose2d(7.784, 1, Rotation2d.fromDegrees(-89.018))
  ),
  false,
  "Blue Five TWO PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(3,4,List.of(
    new Pose2d(7.885, 2.629, Rotation2d.fromDegrees(-150)),
    new Pose2d(5.022, 1.75, Rotation2d.fromDegrees(-140))
  ),
  false,
  "Blue Five TWO PATH_TO_BALL_3"
  );

  private Trajectory PATH_TO_BALL_4_5 = Trajectories.generateTrajectory(3,2,List.of(
    new Pose2d(4.625, 3.3, Rotation2d.fromDegrees(-136)),
    new Pose2d(1.625, 1.625, Rotation2d.fromDegrees(-150))
  ),
  false,
  "Blue Five TWO PATH_TO_BALL_4_5"
  );

  private Trajectory PATH_TO_4_5_SHOT = Trajectories.generateTrajectory(3,2,List.of(
    new Pose2d(2, 1.5, Rotation2d.fromDegrees(200)),
    new Pose2d(4.75, 2.5, Rotation2d.fromDegrees(200))
  ),
  true,
  "Blue Five TWO PATH_TO_4_5_SHOT"
  );

}

