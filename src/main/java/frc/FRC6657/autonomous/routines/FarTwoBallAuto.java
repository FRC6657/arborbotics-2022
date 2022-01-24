// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.SuperStructure;

public class FarTwoBallAuto extends SequentialCommandGroup {

  public FarTwoBallAuto(
    SuperStructure mSuperStructure
  ) {
    addRequirements(
      mSuperStructure
    );
    addCommands(
      mSuperStructure.drivetrain.new TrajectoryFollowerCommand(Trajectories.Two_Ball_Far_1, true) //Move to Pickup Ball #2
        .beforeStarting(
            mSuperStructure.new RunIntakeCommand() //Extend Intake before going to ball #2
        )
        .withTimeout(Trajectories.Two_Ball_Far_1.getTotalTimeSeconds()
      ),
      mSuperStructure.drivetrain.new TrajectoryFollowerCommand(Trajectories.Two_Ball_Far_2, false) // Move to firing position
        .beforeStarting(
          new ParallelCommandGroup(
            mSuperStructure.new StopIntakeCommand(), //Stop intake before going to firing position
            mSuperStructure.flywheel.new setRPMTarget(500) //Set ballpark rpm target before going to firing position
          )
        )
        .withTimeout(Trajectories.Two_Ball_Far_2.getTotalTimeSeconds()),
      new ParallelCommandGroup(
        new InstantCommand(mSuperStructure.vision::toggleLEDs, mSuperStructure) //Toggle LL LEDs
      )
    );
  }
}

  