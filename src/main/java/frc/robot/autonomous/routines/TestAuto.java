// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.Trajectories;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class TestAuto extends SequentialCommandGroup {

  public TestAuto(DrivetrainSubsystem mDrivetrainSubsystem) {
    addRequirements(mDrivetrainSubsystem);
    addCommands(
      mDrivetrainSubsystem.new TrajectoryFollowerCommand(Trajectories.TEST, true).withTimeout(Trajectories.TEST.getTotalTimeSeconds())
    );
  }
}
