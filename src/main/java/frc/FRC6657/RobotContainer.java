// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import javax.management.MBeanServerPermission;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.FRC6657.custom.ArborMath;
import frc.FRC6657.custom.controls.CommandXboxController;
import frc.FRC6657.custom.controls.DriverProfile;
import frc.FRC6657.custom.ctre.IdleMode;
import frc.FRC6657.custom.rev.Blinkin;
import frc.FRC6657.subsystems.SuperStructure;
import frc.FRC6657.subsystems.blinkin.BlinkinSubsystem;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.PickupSubsystem;

@SuppressWarnings("unused")
public class RobotContainer {

  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();
  private final PickupSubsystem mPickupSubsystem = new PickupSubsystem();
  private final BlinkinSubsystem mBlinkinSubsystem = new BlinkinSubsystem();

  // private final ExtensionSubsystem mExtensionSubsystem = new
  // ExtensionSubsystem();
  private final FlywheelSubsystem mFlywheelSubsystem = new FlywheelSubsystem();
  // private final AcceleratorSubsystem mAcceleratorSubsystem = new
  // AcceleratorSubsystem();
  // private final VisionSubsystem mVisionSubsystem = new VisionSubsystem();
  private final LiftSubsystem mLiftSubsystem = new LiftSubsystem();

  private final SuperStructure mSuperStructure = new SuperStructure(
      mDrivetrainSubsystem,
      null, // mPickupSubsystem,
      // mExtensionSubsystem,
      mFlywheelSubsystem,
      null, // mAcceleratorSubsystem,
      null, // mVisionSubsystem.visionSupplier
      null //mLiftSubsystem
  );

  private CommandXboxController mXboxController = new CommandXboxController(0);
  private Joystick mJoystick1 = new Joystick(1);

  private DriverProfile mProfile;

  public RobotContainer() {

    mProfile = getDriver();

    mBlinkinSubsystem = new BlinkinSubsystem();
    mDrivetrainSubsystem = new DrivetrainSubsystem(mProfile);
    mPickupSubsystem = new PickupSubsystem();
    mFlywheelSubsystem = new FlywheelSubsystem();
    mVisionSubsystem = new VisionSubsystem();
    mHoodSubsystem = new HoodSubsystem();
    mAcceleratorSubsystem = new AcceleratorSubsystem();
    mBlinkinSubsystem = new BlinkinSubsystem();


    mSuperStructure = new SuperStructure(
      mDrivetrainSubsystem,
      mPickupSubsystem,
      mFlywheelSubsystem,
      mAcceleratorSubsystem,
      mVisionSubsystem,
      mBlinkinSubsystem
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
          new StartEndCommand(mPickupSubsystem::run, mPickupSubsystem::stop, mPickupSubsystem))
    mXboxController.a().whenHeld(
      new StartEndCommand(
        () -> mPickupSubsystem.set(Constants.Intake.kSpeed),
        mPickupSubsystem::stop,
        mPickupSubsystem
      )
    );

    new JoystickButton(mJoystick1, 2) //button will obviously change
      .whenPressed(
        new ParallelCommandGroup(mSuperStructure.new BlinkinFlywheelNotReady(), new InstantCommand(() -> mFlywheelSubsystem.setRPMTarget(1)))
      ).whenReleased(
        new ParallelCommandGroup(mSuperStructure.new BlinkinFlywheelReady(), mSuperStructure.new ShootCommand())
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
