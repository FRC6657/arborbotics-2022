// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines;

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
        new WaitUntilCommand(intake::BallDetected),
        drivetrain.new TrajectoryFollowerCommand(Trajectories.BALL_TEST_1, true)
      ),
      drivetrain.new TrajectoryFollowerCommand(Trajectories.BALL_TEST_2, false)
    );
  }
}
