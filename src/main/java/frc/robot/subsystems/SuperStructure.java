// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.VisionSupplier;

public class SuperStructure extends SubsystemBase {
  
  public final DrivetrainSubsystem drivetrain;
  public final VisionSupplier visionSupplier;

  public SuperStructure(
    DrivetrainSubsystem drivetrain,
    VisionSupplier visionSupplier
  ) {
    this.drivetrain = drivetrain;
    this.visionSupplier = visionSupplier;
  }

  public void track(){
     if(visionSupplier.hasTarget()){
       
     }
  }

}
