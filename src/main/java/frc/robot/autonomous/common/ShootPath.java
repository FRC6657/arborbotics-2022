// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.common;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class ShootPath extends SequentialCommandGroup {
  public ShootPath(Trajectory path, DrivetrainSubsystem drivetrain, HoodSubsystem hood, FlywheelSubsystem flywheel, VisionSupplier vision) {
    addCommands(
      drivetrain.new TrajectoryFollowerCommand(path),
      drivetrain.new VisionAimAssist()
    );
  }
}
