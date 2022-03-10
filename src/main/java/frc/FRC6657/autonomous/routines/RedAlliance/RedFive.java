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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.custom.ArborSequentialCommandGroup;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.shooter.interpolation.InterpolatingTable;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class RedFive extends ArborSequentialCommandGroup {
  public RedFive(
    DrivetrainSubsystem drivetrain,
    IntakeSubsystem intake,
    ExtensionSubsystem pistons,
    FlywheelSubsystem flywheel,
    AcceleratorSubsystem accelerator,
    HoodSubsystem hood,
    VisionSupplier vision
  ) {
    addReqs(drivetrain, intake, pistons, flywheel, hood, accelerator, vision);
    addCommands(
      new ParallelRaceGroup(
        new WaitUntilCommand(intake::ballDetected), //Cancel Trajectory if ball 2 is detected early
        drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_2, true) //Go to ball #2
      ).beforeStarting(
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
      new TurnAndShoot(drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOT_1, false)),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_3, false)
      .beforeStarting(
        new ParallelCommandGroup(
          new InstantCommand(pistons::extend),
          new InstantCommand(intake::start)
        )
      )
      .andThen(
        new ParallelCommandGroup(
          new InstantCommand(pistons::retract),
          new InstantCommand(intake::stop)
        )
      ),
      new TurnAndShoot(drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOT_2, false)),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_4_5, false)
      .beforeStarting(
        new ParallelCommandGroup(
          new InstantCommand(pistons::extend),
          new InstantCommand(intake::start)
        )
      )
      .andThen(
        new ParallelCommandGroup(
          new InstantCommand(pistons::retract),
          new InstantCommand(intake::stop)
        )
      ),
      new ParallelRaceGroup(
          drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOT_3, false),
          new RunCommand(() -> {
          hood.setAngle(InterpolatingTable.get(vision.getDistance()).hoodAngle);
          System.out.println(vision.getDistance());
          }, hood),
          new RunCommand(() -> flywheel.setRPMTarget(InterpolatingTable.get(vision.getDistance()).rpm), flywheel)
        )
      .andThen(
        new SequentialCommandGroup(
          new WaitUntilCommand(flywheel::atTarget),
          new InstantCommand(accelerator::start)
        ).andThen(
          new ParallelCommandGroup(
            new InstantCommand(accelerator::stop),
            new InstantCommand(flywheel::stop),
            hood.new Home()
          )
        )
      ),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_EXIT, false)
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3,6,List.of(
    new Pose2d(8.91, 6.363, Rotation2d.fromDegrees(91.158)),
    new Pose2d(8.962, 7.26, Rotation2d.fromDegrees(91.158))
  ),
  false,
  "Red Five TWO PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_SHOT_1 = Trajectories.generateTrajectory(1.5,4,List.of(
    new Pose2d(8.962, 7.26, Rotation2d.fromDegrees(95)),
    new Pose2d(8.858, 5.66, Rotation2d.fromDegrees(70))
  ),
  true,
  "Red Five TWO PATH_TO_SHOT_1"
  );

  private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(3,4,List.of(
    new Pose2d(8.858, 6.4, Rotation2d.fromDegrees(0)),
    new Pose2d(10.185, 6.4, Rotation2d.fromDegrees(0)),
    new Pose2d(11.5, 6.4, Rotation2d.fromDegrees(0))
  ),
  false,
  "Red Five TWO PATH_TO_BALL_3"
  );

  private Trajectory PATH_TO_SHOT_2 = Trajectories.generateTrajectory(2,4,List.of(
    new Pose2d(11.5, 7, Rotation2d.fromDegrees(40)),
    new Pose2d(9.8, 5.5, Rotation2d.fromDegrees(40))
  ),
  true,
  "Red Five TWO PATH_TO_SHOT_2"
  );

  private Trajectory PATH_TO_BALL_4_5 = Trajectories.generateTrajectory(1.5,4,List.of(
    new Pose2d(13, 5.5, Rotation2d.fromDegrees(41.68)),
    new Pose2d(15, 6.5, Rotation2d.fromDegrees(41.68))
  ),
  false,
  "Red Five TWO PATH_TO_BALL_4_5"
  );

  private Trajectory PATH_TO_SHOT_3 = Trajectories.generateTrajectory(2,4,List.of(
    new Pose2d(13, 7.5, Rotation2d.fromDegrees(40)),
    new Pose2d(9.7, 5.5, Rotation2d.fromDegrees(40))
  ),
  true,
  "Red Five TWO PATH_TO_SHOT_3"
  );

  private Trajectory PATH_TO_EXIT = Trajectories.generateTrajectory(4,4,List.of(
    new Pose2d(9.8, 5.5, Rotation2d.fromDegrees(60)),
    new Pose2d(10.5, 6.75, Rotation2d.fromDegrees(40))
  ),
  false,
  "Red Five TWO PATH_TO_EXIT"
  );

}
