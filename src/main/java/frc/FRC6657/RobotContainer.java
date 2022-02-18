// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.FRC6657.custom.ArborMath;
import frc.FRC6657.custom.controls.CommandXboxController;
import frc.FRC6657.custom.controls.Deadbander;
import frc.FRC6657.custom.controls.DriverProfile;
import frc.FRC6657.custom.ctre.IdleMode;
import frc.FRC6657.subsystems.SuperStructure;
import frc.FRC6657.subsystems.blinkin.BlinkinSubsystem;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.PickupSubsystem;

public class RobotContainer {

  private final DrivetrainSubsystem mDrivetrainSubsystem;
  private final PickupSubsystem mPickupSubsystem;
  private final SuperStructure mSuperStructure; 

  private CommandXboxController mXboxController = new CommandXboxController(0);
  private Joystick mJoystick1 = new Joystick(1);

  private DriverProfile mProfile;

  public RobotContainer() {

    mProfile = getDriver();

    mDrivetrainSubsystem = new DrivetrainSubsystem(mProfile);
    mPickupSubsystem = new PickupSubsystem();

    mSuperStructure = new SuperStructure(
      mDrivetrainSubsystem,
      mPickupSubsystem
    );


    mDrivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
      mDrivetrainSubsystem.teleopCurvatureDrive(
          -ArborMath.signumPow(mXboxController.getLeftY(), 1.2),
          ArborMath.signumPow(mXboxController.getRightX(), 1.2),
          mXboxController.getRightTriggerAxis() != 0,
          mXboxController.getLeftTriggerAxis() != 0);
    }, mDrivetrainSubsystem));
    

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    new JoystickButton(mJoystick1, 1)
      .whenPressed(
          mSuperStructure.new RunIntakeCommand())
      .whenReleased(
          mSuperStructure.new StopIntakeCommand());

  }

  public Command getAutonomousCommand() {
    return null;
  }

  private DriverProfile getDriver() {
    return new DriverProfile(
        2d, // Max Speed m/s
        90d, // Max Turn deg/s
        3d, // Mod Drive Speed m/s
        80d, // Mod Turn Speed deg/s
        IdleMode.Coast
    );
  }

}
