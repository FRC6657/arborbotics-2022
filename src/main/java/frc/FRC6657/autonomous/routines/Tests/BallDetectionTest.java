// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines.Tests;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;

public class BallDetectionTest extends SequentialCommandGroup {
  public BallDetectionTest(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake
  ) {
    addCommands(
      new ParallelRaceGroup(
        new WaitUntilCommand(intake::ballDetected),
        drivetrain.new TrajectoryFollowerCommand(FORWARD, true)
      ),
      drivetrain.new TrajectoryFollowerCommand(BACKWARD, false)
    );
  }

  public static Trajectory FORWARD = Trajectories.generateTrajectory(
        1,
        1,
        List.of(
            new Pose2d(4, 4, Rotation2d.fromDegrees(0)),
            new Pose2d(5, 4, Rotation2d.fromDegrees(0))
        ),
        false,
        "BallDetection FORWARD"
    );

    public static Trajectory BACKWARD = Trajectories.generateTrajectory(
        1,
        1,
        List.of(
            new Pose2d(5, 4, Rotation2d.fromDegrees(0)),
            new Pose2d(4, 4, Rotation2d.fromDegrees(0))
        ),
        true,
        "BallDetection BACKWARD"
    );

}
