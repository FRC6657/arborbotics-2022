// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.FlywheelSubsystem;

public class FlyWheelTest extends SequentialCommandGroup {

  public FlyWheelTest(FlywheelSubsystem mFlywheelSubsystem) {
    addRequirements(mFlywheelSubsystem);
    addCommands(
      mFlywheelSubsystem.new AdjustRPM(2000)
    );
  }
}
