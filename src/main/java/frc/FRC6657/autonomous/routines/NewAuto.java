// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.SuperStructure;

public class NewAuto extends SequentialCommandGroup {

  public NewAuto(
      SuperStructure mSuperStructure) {
    addRequirements(
        mSuperStructure);
    addCommands(
        mSuperStructure.drivetrain.new TrajectoryFollowerCommand(Trajectories.IntakeTest_1, true)
          .beforeStarting(mSuperStructure.new RunIntakeCommand())
          .withTimeout(Trajectories.IntakeTest_1.getTotalTimeSeconds()),
        mSuperStructure.drivetrain.new TrajectoryFollowerCommand(Trajectories.IntakeTest_2, false)
          .beforeStarting(mSuperStructure.new StopIntakeCommand())
          .withTimeout(Trajectories.IntakeTest_2.getTotalTimeSeconds()),
        mSuperStructure.drivetrain.new TrajectoryFollowerCommand(Trajectories.IntakeTest_3, false)
          .beforeStarting(mSuperStructure.new RunIntakeCommand())
          .withTimeout(Trajectories.IntakeTest_3.getTotalTimeSeconds()),
        mSuperStructure.drivetrain.new TrajectoryFollowerCommand(Trajectories.IntakeTest_4, false)
          .beforeStarting(mSuperStructure.new StopIntakeCommand())
          .withTimeout(Trajectories.IntakeTest_4.getTotalTimeSeconds()));
  }
}
