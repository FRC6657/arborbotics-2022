// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.common;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.Constants;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.shooter.interpolation.InterpolatingTable;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;


/**
 * <p>
 * Aim and set hood angle based on vision measurements.
 * </p>
 * 
 * Note: This will instantly cancel if no target is found
 * 
 */
public class AimRoutine extends ParallelRaceGroup {
  public AimRoutine(DrivetrainSubsystem drivetrain, HoodSubsystem hood, FlywheelSubsystem flywheel, VisionSupplier vision) {
    addCommands(
      new WaitUntilCommand(() -> Math.abs(vision.getYaw()) < Constants.Drivetrain.kTurnCommandTolerance),
      new RunCommand(() -> flywheel.setRPMTarget(InterpolatingTable.get(vision.getDistance()).rpm), flywheel),
      //new RunCommand(() -> hood.setAngle(InterpolatingTable.get(vision.getDistance()).hoodAngle), hood),
      drivetrain.new VisionAimAssist()
    );
  }
}
