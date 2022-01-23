// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

public class SuperStructure extends SubsystemBase {
  
  public final DrivetrainSubsystem drivetrain;
  public final FlywheelSubsystem flywheel;

  public SuperStructure(
    DrivetrainSubsystem drivetrain,
    FlywheelSubsystem flywheel
  ) {
    this.drivetrain = drivetrain;
    this.flywheel = flywheel;
  }

  public class ShootCommand extends SequentialCommandGroup{

    public ShootCommand(){
      addRequirements(SuperStructure.this);
      addCommands(
        
      );
    }
  }
}
