// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines.Tests;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;

public class PoseTest extends SequentialCommandGroup {
  public PoseTest(
    DrivetrainSubsystem drivetrain
  ) {
    addCommands(
      drivetrain.new TrajectoryFollowerCommand(SAMPLE, true)
    );
  }

  public static Trajectory SAMPLE = Trajectories.generateTrajectory(
    1,
    1,
    List.of(
        new Pose2d(5, 5, Rotation2d.fromDegrees(90)),
        new Pose2d(5.01, 5, Rotation2d.fromDegrees(90))
    ),
    false,
    "PoseTest SAMPLE"
);

}
