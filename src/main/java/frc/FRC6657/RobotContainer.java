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
import frc.FRC6657.custom.rev.Blinkin;
import frc.FRC6657.subsystems.blinkin.BlinkinSubsystem;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.lift.LiftSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem;

@SuppressWarnings("unused")
public class RobotContainer {

  private CommandXboxController mXboxController = new CommandXboxController(0);
  private Joystick mJoystick = new Joystick(1);

  private DriverProfile mProfile;

  public final AcceleratorSubsystem accelerator;
  public final BlinkinSubsystem blinkin;
  public final DrivetrainSubsystem drivetrain;
  public final FlywheelSubsystem flywheel;
  public final HoodSubsystem hood;
  public final IntakeSubsystem intake;
  public final LiftSubsystem lift;
  public final VisionSubsystem vision;

  public RobotContainer() {

    mProfile = getDriver();

    accelerator = new AcceleratorSubsystem();
    blinkin = new BlinkinSubsystem();
    drivetrain = new DrivetrainSubsystem(mProfile);
    flywheel = new FlywheelSubsystem();
    hood = new HoodSubsystem();
    intake = new IntakeSubsystem();
    lift = new LiftSubsystem();
    vision = new VisionSubsystem();

    drivetrain.setDefaultCommand(new RunCommand(() -> {
      drivetrain.teleopCurvatureDrive(
          -ArborMath.signumPow(mXboxController.getLeftY(), 1.2),
          ArborMath.signumPow(mXboxController.getRightX(), 1.2),
          mXboxController.getRightTrigger(),
          mXboxController.getLeftTrigger());
    }, drivetrain));
    
    configureButtonBindings();

  }

  private void configureButtonBindings() {
    
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
