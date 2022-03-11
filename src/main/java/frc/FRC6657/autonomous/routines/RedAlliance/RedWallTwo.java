// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.autonomous.common.AimRoutine;
import frc.FRC6657.autonomous.common.FireRoutine;
import frc.FRC6657.autonomous.common.IntakePath;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
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
      new InstantCommand(() -> drivetrain.resetPoseEstimator(PATH_TO_BALL_2.getInitialPose()), drivetrain), //Reset Position
      new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons), //Intake Red 2
      new AimRoutine(drivetrain, hood, flywheel, vision), //Aim
      new FireRoutine(flywheel, hood, accelerator, 0.5), //Fire Red 2
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_TAXI) //Ensure Taxi
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(1, 2, List.of(
    new Pose2d(8.85, 6.36, Rotation2d.fromDegrees(90.7)),
    new Pose2d(8.93, 7.34, Rotation2d.fromDegrees(90))
  ), false, 
  "Red Wall Two PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_TAXI = Trajectories.generateTrajectory(1, 2, List.of(
    new Pose2d(8.93, 7.34, Rotation2d.fromDegrees(180)),
    new Pose2d(11,7.5, Rotation2d.fromDegrees(180))
  ), true, 
  "Red Wall Two PATH_TO_TAXI"
  );

}
