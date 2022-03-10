// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.common;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;

public class FireRoutine extends SequentialCommandGroup {
  public FireRoutine(FlywheelSubsystem flywheel, HoodSubsystem hood, AcceleratorSubsystem accelerator, double shotDurration) {
    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new InstantCommand(accelerator::start),
          new WaitCommand(shotDurration),
          new InstantCommand(accelerator::stop)
        ),
        new InstantCommand(),
        () -> (flywheel.atTarget() && hood.atTarget())
      ).andThen(
        new ParallelCommandGroup(
          hood.new Home(),
          new InstantCommand(flywheel::stop, flywheel)
        )
      )      
    );
  }
}
