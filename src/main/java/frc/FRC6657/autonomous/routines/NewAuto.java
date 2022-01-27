// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.SuperStructure;

public class NewAuto extends SequentialCommandGroup {

  public NewAuto(
    SuperStructure mSuperStructure
  ) {
    addRequirements(
      mSuperStructure
    );
    addCommands(
      mSuperStructure.drivetrain.new TrajectoryFollowerCommand(Trajectories.IntakeTest, true) //Move to Pickup Ball #2
        .beforeStarting(
            mSuperStructure.new RunIntakeCommand() //Extend Intake before going to ball #2
        )
        .withTimeout(Trajectories.IntakeTest.getTotalTimeSeconds())
    );
  }
}

