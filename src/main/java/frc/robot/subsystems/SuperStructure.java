// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.PickupSubsystem;
import frc.robot.subsystems.intake.PivotSubsystem;
import frc.robot.subsystems.pneumatics.PneumaticsController;

public class SuperStructure extends SubsystemBase {
  
  public final DrivetrainSubsystem drivetrain;
  public final PickupSubsystem pickup;
  public final PivotSubsystem pivot;
  public final PneumaticsController pneumatics;

  public SuperStructure(
    DrivetrainSubsystem drivetrain,
    PickupSubsystem pickup,
    PivotSubsystem pivot,
    PneumaticsController pneumatics
  ) {
    this.drivetrain = drivetrain;
    this.pickup = pickup;
    this.pivot = pivot;
    this.pneumatics = pneumatics;
  }
}
