// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.common;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;

/**
 * <p>
 * Path that extends the intake at the beginning at the beginning and retracts it at the end.
 * </p>
 * It also cancels if a ball is detected before the path has ended
 * 
 */
public class IntakePath extends ParallelRaceGroup {
  public IntakePath(Trajectory path, DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ExtensionSubsystem pistons) {
    addCommands(
        new WaitUntilCommand(intake::ballDetected),
        drivetrain.new TrajectoryFollowerCommand(path)
        .beforeStarting(
          new ParallelCommandGroup(
            new InstantCommand(pistons::extend, pistons),
            new InstantCommand(intake::start, intake)
          )
        )
        .andThen(
          new ParallelCommandGroup(
            new InstantCommand(pistons::extend, pistons),
            new InstantCommand(intake::start, intake)
          )
        )
    );
  }
}
