// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.routines.test;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.Trajectories;
import frc.robot.autonomous.common.IntakePath;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class BlueFive extends SequentialCommandGroup{
  /** Creates a new BlueFive. */
  public BlueFive(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake,
    IntakePistonsSubsystem pistons
  ) {
    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(PATH_TO_BALL_2.getInitialPose()), drivetrain), //Reset Position
      new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons), //Intake Blue 2
      new IntakePath(PATH_TO_BALL_3, drivetrain, intake, pistons), // Intake Blue 3
      new IntakePath(PATH_TO_BALL_4_5, drivetrain, intake, pistons), //Intake Blue 4 & 5
      new IntakePath(PATH_TO_4_5_SHOT, drivetrain, intake, pistons)
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3,2,List.of(
    new Pose2d(7.821, 1.922, Rotation2d.fromDegrees(-89.018)),
    new Pose2d(7.784, 0.9, Rotation2d.fromDegrees(-89.018))
  ),
  false,
  "Blue Five TWO PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(3,2,List.of(
    new Pose2d(7.885, 2.629, Rotation2d.fromDegrees(-150)),
    new Pose2d(5.022, 3, Rotation2d.fromDegrees(-140))
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

