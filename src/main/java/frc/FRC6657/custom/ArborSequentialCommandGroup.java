// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.custom;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.Constants;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.shooter.interpolation.InterpolatingTable;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArborSequentialCommandGroup extends SequentialCommandGroup {
  /** Creates a new ArborSequentialCommandGroup. */
  DrivetrainSubsystem drivetrain;
  IntakeSubsystem intake;
  ExtensionSubsystem pistons;
  FlywheelSubsystem flywheel;
  HoodSubsystem hood;
  AcceleratorSubsystem accelerator;
  VisionSupplier vision;

  public void addReqs(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake,
    ExtensionSubsystem pistons,
    FlywheelSubsystem flywheel,
    HoodSubsystem hood,
    AcceleratorSubsystem accelerator,
    VisionSupplier vision
  ){
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.pistons = pistons;
    this.flywheel = flywheel;
    this.hood = hood;
    this.accelerator = accelerator;
    this.vision = vision;
  }

  public class TurnAndShoot extends SequentialCommandGroup {
    public TurnAndShoot(Command cmd) {
      addCommands(
        cmd,
        new TurnAndShoot()
      );
    }
    public TurnAndShoot() {
      addCommands(
        new ParallelRaceGroup(
                new WaitUntilCommand(() -> Math.abs(vision.getYaw()) < Constants.Drivetrain.kTurnCommandTolerance),
                drivetrain.new VisionAimAssist(),
                new RunCommand(
                    () -> hood.setAngle(InterpolatingTable.get(vision.getDistance()).hoodAngle),
                    hood
                ),
                new RunCommand(
                    () -> flywheel.setRPMTarget(InterpolatingTable.get(vision.getDistance()).rpm),
                    flywheel
                )
            ),
            new WaitUntilCommand(() -> (flywheel.atTarget() && hood.atTarget())),
            new SequentialCommandGroup(
                new InstantCommand(accelerator::start, accelerator),
                new WaitCommand(1),
                new InstantCommand(accelerator::stop, accelerator)
            ).andThen(
                new ParallelCommandGroup(
                    hood.new Home(),
                    new InstantCommand(() -> flywheel.setRPMTarget(0))
                )
            )
        );
    }
  }

}
