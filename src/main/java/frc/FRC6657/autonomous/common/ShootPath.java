// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.common;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;
public class ShootPath extends SequentialCommandGroup {
  public ShootPath(Trajectory path, DrivetrainSubsystem drivetrain, HoodSubsystem hood, FlywheelSubsystem flywheel, VisionSupplier vision) {
    addCommands(
      drivetrain.new TrajectoryFollowerCommand(path),
      new AimRoutine(drivetrain, hood, flywheel, vision)
    );
  }
}
