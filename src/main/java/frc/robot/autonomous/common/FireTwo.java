// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class FireTwo extends SequentialCommandGroup {
  public FireTwo(FlywheelSubsystem flywheel, HoodSubsystem hood, AcceleratorSubsystem accelerator, IntakePistonsSubsystem pistons, double rpm, double angle) {
    addCommands(
      new SequentialCommandGroup(
          new ParallelCommandGroup(
            new InstantCommand(() -> flywheel.setTargetRPM(rpm), flywheel),
            new InstantCommand(() -> hood.setTargetAngle(angle), hood)
          ),
          new WaitUntilCommand(() -> flywheel.ready()),
          new InstantCommand(pistons::extend, pistons),
          new WaitCommand(0.25),
          new RunCommand(accelerator::start, accelerator).withInterrupt(() -> flywheel.shotDetector()).withTimeout(2),
          new InstantCommand(pistons::retract),
          new InstantCommand(() -> accelerator.set(-1)),
          new WaitCommand(0.25),

          new InstantCommand(accelerator::stop),
          new WaitUntilCommand(() -> flywheel.ready()),
          new RunCommand(accelerator::start).withInterrupt(() -> flywheel.shotDetector()).withTimeout(2),
          new ParallelCommandGroup(
            new InstantCommand(flywheel::stop, flywheel),
            new InstantCommand(hood::stop, hood),
            new InstantCommand(accelerator::stop, accelerator),
            new InstantCommand(pistons::retract, pistons)
          )
      )
    );
  }
}
