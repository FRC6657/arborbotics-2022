// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();
  public RobotContainer() {
    configureButtonBindings();
  }
  private void configureButtonBindings() {}
  public Command getAutonomousCommand() {
    return null;
  }
}
