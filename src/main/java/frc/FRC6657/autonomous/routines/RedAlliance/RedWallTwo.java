// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.Constants;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.shooter.interpolation.InterpolatingTable;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;
public class RedWallTwo extends SequentialCommandGroup {
  public RedWallTwo(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake,
    ExtensionSubsystem pistons,
    FlywheelSubsystem flywheel,
    AcceleratorSubsystem accelerator,
    HoodSubsystem hood,
    VisionSupplier vision
  ) {
    addCommands(
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_1, true)
      .beforeStarting(
        new ParallelCommandGroup( //Prepare Intake to pickup ball #2
          new InstantCommand(pistons::extend), 
          new InstantCommand(intake::start)
        )
      )
      .andThen(
        new ParallelCommandGroup( //Retract Intake After Ball #2
          new InstantCommand(pistons::retract),
          new InstantCommand(intake::stop)
        )
      ),
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
  private Trajectory PATH_TO_BALL_1 = Trajectories.generateTrajectory(1, 2, List.of(
    new Pose2d(8.85, 6.36, Rotation2d.fromDegrees(90.7)),
    new Pose2d(8.93, 7.34, Rotation2d.fromDegrees(90))
  ), false, 
  "Red Wall 2 PATH_TO_BALL"
);
}
