// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.    subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.Interpolation.InterpolatingTable;

public class FireRoutine extends SequentialCommandGroup {
  public FireRoutine(FlywheelSubsystem flywheel, HoodSubsystem hood, AcceleratorSubsystem accelerator, VisionSupplier vision, IntakePistonsSubsystem pistons) {
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(() -> flywheel.setTargetRPM(InterpolatingTable.get(vision.getDistance()).rpm), flywheel),
        new InstantCommand(() -> hood.setTargetAngle(InterpolatingTable.get(vision.getDistance()).hoodAngle), hood)
      ),
      new ParallelCommandGroup(
        new WaitUntilCommand(flywheel::ready),
        new WaitUntilCommand(hood::ready)
      ),
      new InstantCommand(pistons::extend, pistons),
      new InstantCommand(accelerator::start, accelerator),
      new WaitCommand(2)
    );
  }
}
