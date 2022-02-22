// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.subsystems.blinkin.BlinkinSubsystem;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.lift.LiftSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class SuperStructure extends SubsystemBase {
  
  public final AcceleratorSubsystem accelerator;
  public final BlinkinSubsystem blinkin;
  public final DrivetrainSubsystem drivetrain;
  public final FlywheelSubsystem flywheel;
  public final HoodSubsystem hood;
  public final IntakeSubsystem intake;
  public final LiftSubsystem lift;
  public final VisionSupplier vision;

  public SuperStructure(
    AcceleratorSubsystem accelerator,
    BlinkinSubsystem blinkin,
    DrivetrainSubsystem drivetrain,
    FlywheelSubsystem flywheel,
    HoodSubsystem hood,
    IntakeSubsystem intake,
    LiftSubsystem lift,
    VisionSupplier vision
  ) {
    this.accelerator = accelerator;
    this.blinkin = blinkin;
    this.drivetrain = drivetrain;
    this.flywheel = flywheel;
    this.hood = hood;
    this.intake = intake;
    this.lift = lift;
    this.vision = vision;
  }

    

  }
