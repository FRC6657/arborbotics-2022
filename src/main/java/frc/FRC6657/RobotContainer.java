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
import frc.FRC6657.custom.controls.ControlStyle;
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

  private XboxController mXboxController = new XboxController(0);
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

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    if(mProfile.kStyle == ControlStyle.Curvature){
      mDrivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
        mDrivetrainSubsystem.teleopCurvatureDrive(
          -Math.copySign(mProfile.mController.getRawAxis(mProfile.kDriveAxis), Math.pow(Deadbander.applyLinearScaledDeadband(mProfile.mController.getRawAxis(mProfile.kTurnAxis), 0.1), 2)),
          Math.copySign(mProfile.mController.getRawAxis(mProfile.kTurnAxis),Math.pow(Deadbander.applyLinearScaledDeadband(mProfile.mController.getRawAxis(mProfile.kTurnAxis), 0.1), 2)),
          mProfile.mController.getRawAxis(mProfile.kQuickturnBtn) != 0,
          mProfile.mController.getRawAxis(mProfile.kSpeedModBtn) != 0
        );
      }, mDrivetrainSubsystem));
    }

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
        mXboxController, // Controller
        Axis.kLeftY.value, // Drive Axis
        Axis.kRightX.value, // Turn Axis
        ControlStyle.Curvature,
        2, // Max Speed m/s
        90, // Max Turn deg/s
        IdleMode.Coast,
        2, // Speed Mod BTN
        3, // Mod Drive Speed m/s
        80, // Mod Turn Speed deg/s
        3 // Quickturn BTN
    );
  }

}
