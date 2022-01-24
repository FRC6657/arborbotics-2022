// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.SuperStructure;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;

public class FarTwoBallAuto extends SequentialCommandGroup {

  public FarTwoBallAuto(DrivetrainSubsystem mDrivetrainSubsystem, SuperStructure mSuperStructure, FlywheelSubsystem mFlywheelSubsystem) {
    addRequirements(mDrivetrainSubsystem, mSuperStructure, mFlywheelSubsystem);
    addCommands(
      mDrivetrainSubsystem.new TrajectoryFollowerCommand(Trajectories.Two_Ball_Far_1, true) //Move to Pickup Ball #2
        .beforeStarting(
          mSuperStructure.new RunIntakeCommand() //Extend Intake Before Moving
        )
        .withTimeout(Trajectories.Two_Ball_Far_1.getTotalTimeSeconds()
      ),
      mDrivetrainSubsystem.new TrajectoryFollowerCommand(Trajectories.Two_Ball_Far_2, false) //Move to firing position
        .beforeStarting(
          new ParallelCommandGroup(
            mSuperStructure.new StopIntakeCommand(), //Stop and retract intake
            mFlywheelSubsystem.new setRPMTarget(500) //Set Ballpark RPM
          )
        )
        .withTimeout(Trajectories.Two_Ball_Far_2.getTotalTimeSeconds())
    );
  }
}

