// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.FRC6657.autonomous.routines.FarTwoBallAuto;
import frc.FRC6657.autonomous.routines.NewAuto;
import frc.FRC6657.custom.controls.Deadbander;
import frc.FRC6657.subsystems.SuperStructure;
import frc.FRC6657.subsystems.blinkin.BlinkinSubsystem;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
//import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.PickupSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
//import frc.FRC6657.subsystems.vision.VisionSubsystem;

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

  private final SuperStructure mSuperStructure = new SuperStructure(
      mDrivetrainSubsystem,
      mPickupSubsystem,
      mFlywheelSubsystem
  );

  private final XboxController mDriver = new XboxController(0);

  private final SlewRateLimiter mAccelLimit = new SlewRateLimiter(Constants.Drivetrain.kMaxAccel);

  public RobotContainer() {

    mDrivetrainSubsystem.setDefaultCommand(
    mDrivetrainSubsystem.new DriveCommand(
    () ->
    -mAccelLimit.calculate(Deadbander.applyLinearScaledDeadband(mDriver.getLeftY(),0.1)),
    () -> Deadbander.applyLinearScaledDeadband(mDriver.getRightX(),0.15),
    () -> mDriver.getRightBumper()
    )
    );

    configureButtonBindings();

  }

  private void configureButtonBindings() {
    new JoystickButton(mDriver, XboxController.Button.kA.value)
        .whenPressed(mSuperStructure.new RunIntakeCommand())
        .whenReleased(mSuperStructure.new StopIntakeCommand());

    new JoystickButton(mDriver, XboxController.Button.kB.value)
      .whenPressed(new InstantCommand(mFlywheelSubsystem::run, mFlywheelSubsystem))
      .whenReleased(new InstantCommand(mFlywheelSubsystem::stop, mFlywheelSubsystem));

  }

  public Command getAutonomousCommand() {
    // return new FarTwoBallAuto(mSuperStructure);
    return new NewAuto(mSuperStructure);
  }
}
