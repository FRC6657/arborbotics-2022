// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.routines.TestAuto;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class RobotContainer {
  
  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController mDriver = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();

    mDrivetrainSubsystem.setDefaultCommand(
      mDrivetrainSubsystem.new DriveCommand(
        () -> -deadband(mDriver.getLeftY(),0.1),
        () -> deadband(mDriver.getRightX(),0.1),
        () -> mDriver.getRightBumper()
      )
    );

  }
  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return new TestAuto(mDrivetrainSubsystem);
  }

  public double deadband(double input, double threshold){
    if(Math.abs(input)<threshold){
      return 0;
    } else{
      return (input-(Math.abs(input)/input)*threshold ) / (1.0 - threshold);
    }
  }

}
