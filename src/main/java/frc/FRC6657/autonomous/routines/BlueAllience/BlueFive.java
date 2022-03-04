// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines.BlueAllience;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;

public class BlueFive extends SequentialCommandGroup {
  /** Creates a new BlueFive. */
  public BlueFive(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake,
    ExtensionSubsystem pistons,
    FlywheelSubsystem flywheel,
    AcceleratorSubsystem accelerator
  ) {
    addCommands();
  }
}
