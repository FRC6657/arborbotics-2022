// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.FRC6657.custom.ArborMath;
import frc.FRC6657.custom.controls.CommandXboxController;
import frc.FRC6657.custom.controls.DriverProfile;
import frc.FRC6657.subsystems.SuperStructure;
import frc.FRC6657.subsystems.blinkin.BlinkinSubsystem;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.PickupSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
@SuppressWarnings("unused")
public class RobotContainer {

  private BlinkinSubsystem mBlinkinSubsystem;
  private DrivetrainSubsystem mDrivetrainSubsystem;
  private PickupSubsystem mPickupSubsystem;
  private AcceleratorSubsystem mAcceleratorSubsystem;
  private FlywheelSubsystem mFlywheelSubsystem;
  private HoodSubsystem mHoodSubsystem;
  private SuperStructure mSuperStructure; 

  private CommandXboxController mXboxController = new CommandXboxController(0);
  private Joystick mJoystick1 = new Joystick(1);

  private DriverProfile mProfile;

  public RobotContainer() {

    mProfile = getDriver();

    mBlinkinSubsystem = new BlinkinSubsystem();
    mDrivetrainSubsystem = new DrivetrainSubsystem(mProfile);
    mPickupSubsystem = new PickupSubsystem();
    mAcceleratorSubsystem = new AcceleratorSubsystem();
    mFlywheelSubsystem = new FlywheelSubsystem();
    mHoodSubsystem = new HoodSubsystem();

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

    mXboxController.a().whenHeld(
      new StartEndCommand(
        () -> mPickupSubsystem.set(Constants.Intake.kSpeed),
        mPickupSubsystem::stop,
        mPickupSubsystem
      )
    );

  }

  public Command getAutonomousCommand() {
    return null;
  }

  private DriverProfile getDriver() {
    return new DriverProfile(
        5d, // Max Speed m/s
        90d, // Max Turn deg/s
        3d, // Mod Drive Speed m/s
        80d // Mod Turn Speed deg/s
    );
  }

}
