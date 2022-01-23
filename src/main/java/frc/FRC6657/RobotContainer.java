// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.FRC6657.autonomous.routines.TestAuto;
import frc.FRC6657.custom.controls.Deadbander;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.PickupSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;

public class RobotContainer {
  
  private final DrivetrainSubsystem mDrivetrainSubsystem = new DrivetrainSubsystem();
  private final PickupSubsystem mPickupSubsystem = new PickupSubsystem();
  private final ExtensionSubsystem mExtensionSubsystem = new ExtensionSubsystem();
  private final FlywheelSubsystem mFlywheelSubsystem = new FlywheelSubsystem();

  private final XboxController mDriver = new XboxController(0);

  private final SlewRateLimiter mAccelLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccel);

  public RobotContainer() {

    mDrivetrainSubsystem.setDefaultCommand(
      mDrivetrainSubsystem.new DriveCommand(
        () -> -mAccelLimit.calculate(Deadbander.applyLinearScaledDeadband(mDriver.getLeftY(),0.1)),
        () -> Deadbander.applyLinearScaledDeadband(mDriver.getRightX(),0.1),
        () -> mDriver.getRightBumper()
      )
    );

    configureButtonBindings();

  }
  private void configureButtonBindings() {
    new JoystickButton(mDriver, XboxController.Button.kA.value)
      .whenPressed(new InstantCommand(mPickupSubsystem::run, mPickupSubsystem))
      .whenReleased(new InstantCommand(mPickupSubsystem::stop, mPickupSubsystem));

    new JoystickButton(mDriver, XboxController.Button.kB.value)
      .whenPressed(new InstantCommand(mExtensionSubsystem::toggleState, mExtensionSubsystem));

  }

  public Command getAutonomousCommand() {
    return new TestAuto(mDrivetrainSubsystem);
  }
}
