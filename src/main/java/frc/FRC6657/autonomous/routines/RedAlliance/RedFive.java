// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines.RedAlliance;

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

public class RedFive extends SequentialCommandGroup{
  public RedFive(
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
      new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons), //Intake Red 2
      new AimRoutine(drivetrain, hood, flywheel, vision).withTimeout(1.5), //Aim
      new FireRoutine(flywheel, hood, accelerator, 0.5).withTimeout(1.5), //Shoot Red 1 & 2
      new IntakePath(PATH_TO_BALL_3, drivetrain, intake, pistons), //Intake Red 3
      new AimRoutine(drivetrain, hood, flywheel, vision).withTimeout(1.5), //Aim
      new FireRoutine(flywheel, hood, accelerator, 0.5).withTimeout(1.5), //Fire Red 3
      new IntakePath(PATH_TO_BALL_4_5, drivetrain, intake, pistons), //Intake Red 4 & 5
      new ShootPath(PATH_TO_4_5_SHOT, drivetrain, hood, flywheel, vision), //Move to Firing Position
      new FireRoutine(flywheel, hood, accelerator, 0.5).withTimeout(1.5) //Fire Red 4 & 5
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(2,3,List.of(
    new Pose2d(8.91, 6, Rotation2d.fromDegrees(91.158)),
    new Pose2d(8.962, 7.4, Rotation2d.fromDegrees(91.158))
  ),
  false,
  "Red Five TWO PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(3,4,List.of(
    new Pose2d(8.25, 6.45, Rotation2d.fromDegrees(0)),
    new Pose2d(11.5, 6.4, Rotation2d.fromDegrees(35))
  ),
  false,
  "Red Five TWO PATH_TO_BALL_3"
  );

  private Trajectory PATH_TO_BALL_4_5 = Trajectories.generateTrajectory(3,2,List.of(
    new Pose2d(13.125, 5.25, Rotation2d.fromDegrees(40)),
    new Pose2d(14.8, 6.7, Rotation2d.fromDegrees(40))
  ),
  false,
  "Red Five TWO PATH_TO_BALL_4_5"
  );

  private Trajectory PATH_TO_4_5_SHOT = Trajectories.generateTrajectory(3,4,List.of(
    new Pose2d(15, 6.75, Rotation2d.fromDegrees(26)),
    new Pose2d(11.5, 5.75, Rotation2d.fromDegrees(13))
  ),
  true,
  "Red Five TWO PATH_TO_4_5_SHOT"
  );

}
