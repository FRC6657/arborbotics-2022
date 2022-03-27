// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.common;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * <p>
 * Path that extends the intake at the beginning at the beginning and retracts it at the end.
 * </p>
 * It also cancels if a ball is detected before the path has ended
 * 
 */
public class IntakePath extends ParallelRaceGroup {
  public IntakePath(Trajectory path, DrivetrainSubsystem drivetrain, IntakeSubsystem intake, IntakePistonsSubsystem pistons) {
    addCommands(
        drivetrain.new TrajectoryFollowerCommand(path)
        .beforeStarting(
          new ParallelCommandGroup(
            new InstantCommand(pistons::extend, pistons),
            new InstantCommand(intake::start, intake)
          )
        )
        .andThen(
          new ParallelCommandGroup(
            new InstantCommand(pistons::retract, pistons),
            new InstantCommand(intake::stop, intake)
          )
        )
    );
  }
}
