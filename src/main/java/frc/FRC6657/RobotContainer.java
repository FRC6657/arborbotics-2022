// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

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

  private Joystick mJoystick1 = new Joystick(0);
  private XboxController mXboxController = new XboxController(0);

  //private String driver = "TieuTam";
  private String driver = "Andrew";

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
          -Deadbander.applyLinearScaledDeadband(mProfile.mController.getRawAxis(mProfile.kDriveAxis), 0.1),
          Deadbander.applyLinearScaledDeadband(mProfile.mController.getRawAxis(mProfile.kTurnAxis), 0.1),
          mProfile.mController.getRawAxis(mProfile.kQuickturnBtn) != 0,
          mProfile.mController.getRawAxis(mProfile.kSpeedModBtn) != 0
        );
      }, mDrivetrainSubsystem));
    }

    if(mProfile.kStyle == ControlStyle.Arcade){
      mDrivetrainSubsystem.setDefaultCommand(new RunCommand(() -> {
        mDrivetrainSubsystem.teleopArcadeDrive(
          -Deadbander.applyLinearScaledDeadband(mProfile.mController.getRawAxis(mProfile.kDriveAxis), 0.1),
          Deadbander.applyLinearScaledDeadband(mProfile.mController.getRawAxis(mProfile.kTurnAxis), 0.1),
          mProfile.mController.getRawButton(mProfile.kSpeedModBtn)
        );
      }, mDrivetrainSubsystem));
    }

    switch(driver){
      case "TieuTam":
        new JoystickButton(mXboxController, 6)
          .whenHeld(
            mSuperStructure.new RunIntakeCommand()
          )
          .whenReleased(
            mSuperStructure.new StopIntakeCommand()
          );
      case "Andrew":
        new JoystickButton(mJoystick1, 1)
        .whenHeld(
          mSuperStructure.new RunIntakeCommand()
        )
        .whenReleased(
          mSuperStructure.new StopIntakeCommand()
        );
    }

  }

  public Command getAutonomousCommand() {
    return null;
  }

  private DriverProfile getDriver() {
    switch(driver){
      default:
        return
          new DriverProfile(
            mJoystick1,
            AxisType.kY.value,
            AxisType.kTwist.value,
            ControlStyle.Arcade,
            Constants.Drivetrain.kMaxAttainableSpeed,
            Constants.Drivetrain.kMaxAttainableTurnRate,
            IdleMode.Coast,
            1,
            1d,
            1d
          );
      case "TieuTam":
        return
          new DriverProfile(
            mJoystick1,//Controller
            AxisType.kY.value,//Drive Axis
            AxisType.kTwist.value,//Turn Axis
            ControlStyle.Arcade,
            Constants.Drivetrain.kMaxAttainableSpeed * 0.65, //Max Speed m/s
            Constants.Drivetrain.kMaxAttainableTurnRate * 0.55, //Max Turn deg/s
            IdleMode.Brake,
            1,//Speed Mod BTN
            Constants.Drivetrain.kMaxAttainableSpeed * 0.8, //Mod Drive Speed m/s
            Constants.Drivetrain.kMaxAttainableTurnRate * 0.7 //Mod Turn Speed deg/s
          );
          
      case "Andrew":
        return
          new DriverProfile(
            mXboxController,//Controller
            Axis.kLeftY.value, //Drive Axis
            Axis.kRightX.value, //Turn Axis
            ControlStyle.Curvature,
            3.5, //Max Speed m/s
            360, //Max Turn deg/s
            IdleMode.Coast,
            2, //Speed Mod BTN
            1.5, //Mod Drive Speed m/s
            45, //Mod Turn Speed deg/s
            3 //Quickturn BTN
          );

    }
  }

}
