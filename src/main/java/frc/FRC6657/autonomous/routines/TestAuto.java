// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;

public class TestAuto extends SequentialCommandGroup {

  public TestAuto(DrivetrainSubsystem mDrivetrainSubsystem) {
    addRequirements(mDrivetrainSubsystem);
    addCommands(
      mDrivetrainSubsystem.new TrajectoryFollowerCommand(Trajectories.Two_Ball_Far_1, true)
    );
  }
}
