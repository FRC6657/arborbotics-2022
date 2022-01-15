// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.routines.TestAuto;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.PickupSubsystem;

public class RobotContainer {
  
  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();
  private final PickupSubsystem mPickupSubsystem = new PickupSubsystem();

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
  private void configureButtonBindings() {
    //new JoystickButton(mDriver, XboxController.Button.kA.value).whenHeld(mPickupSubsystem.new PickupCommand());
  }

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
