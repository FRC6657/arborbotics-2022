// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.FRC6657.custom.controls.Deadbander;
import frc.FRC6657.custom.controls.DriverProfile;
import frc.FRC6657.subsystems.SuperStructure;
import frc.FRC6657.subsystems.blinkin.BlinkinSubsystem;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.PickupSubsystem;

public class RobotContainer {

  private DrivetrainSubsystem mDrivetrainSubsystem;
  private final PickupSubsystem mPickupSubsystem = new PickupSubsystem();
  private final BlinkinSubsystem mBlinkinSubsystem = new BlinkinSubsystem();

  private final SuperStructure mSuperStructure = new SuperStructure(
      mDrivetrainSubsystem,
      mPickupSubsystem
  );

  private Joystick mJoystick1;
  private Joystick mJoystick2;
  private XboxController mXboxController;

  private String driver = "Andrew";

  public RobotContainer() {

    switch(driver){
      case "Default":
        mDrivetrainSubsystem = new DrivetrainSubsystem(
          new DriverProfile(
            Constants.Drivetrain.kMaxAttainableSpeed,
            Constants.Drivetrain.kMaxAttainableTurnRate
          )
        );

      case "TieuTam":
        mDrivetrainSubsystem = new DrivetrainSubsystem(
          new DriverProfile(
            Constants.Drivetrain.kMaxAttainableSpeed * 0.65,
            Constants.Drivetrain.kMaxAttainableTurnRate * 0.55
          )
        );
      case "Andrew":
        mDrivetrainSubsystem = new DrivetrainSubsystem(
          new DriverProfile(
            3,
            360
          )
        );
    }

    configureButtonBindings(driver);
  }

  private void configureButtonBindings(String driver) {

    switch(driver){
      case "Default":
        mJoystick1 = new Joystick(0);
        mXboxController = new XboxController(1);
        mDrivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
          mDrivetrainSubsystem.teleopArcadeDrive(
            -mJoystick1.getY() * 0.65,
            mJoystick1.getTwist() * 0.55);
        }, mDrivetrainSubsystem));
      case "TieuTam":
        mJoystick1 = new Joystick(0);
        mXboxController = new XboxController(1);
      case "Andrew":
        mXboxController = new XboxController(0);
        mJoystick1 = new Joystick(1);
        mDrivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
          mDrivetrainSubsystem.teleopCurvatureDrive(
            -Deadbander.applyLinearScaledDeadband(mXboxController.getLeftY(), 0.1),
            Deadbander.applyLinearScaledDeadband(mXboxController.getRightX(), 0.1),
            mXboxController.getRightTriggerAxis() != 0
          );
        }, mDrivetrainSubsystem));
    }
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
