package frc.robot.autonomous.routines.BlueAllience;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.autonomous.Trajectories;
import frc.robot.autonomous.common.IntakePath;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RedFenderFive extends SequentialCommandGroup {
  public RedFenderFive(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, IntakePistonsSubsystem pistons, AcceleratorSubsystem accelerator) {
    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(Constants.Field.RED_FENDER_1)),
      //Shoot
      new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons),
      new IntakePath(PATH_TO_BALL_3, drivetrain, intake, pistons),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_FENDER),
      //Shoot
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_4_5)
        .beforeStarting(
          new ParallelCommandGroup(
            new InstantCommand(intake::start, intake),
            new InstantCommand(pistons::extend, pistons)
          )
        ).andThen(
          new SequentialCommandGroup(
            new StartEndCommand(
              accelerator::start, accelerator::stop, accelerator
            ).withTimeout(0.5),
            new ParallelCommandGroup(
              new InstantCommand(intake::stop, intake),
              new InstantCommand(pistons::retract, pistons)
            )
          )
        ),
      new WaitCommand(0.5),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_FENDER_2)
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3, 2, List.of(
    Constants.Field.RED_FENDER_1,
    new Pose2d(Constants.Field.RED_1.minus(new Translation2d(0, 0.5)), Rotation2d.fromDegrees(90))
  ), 
  false, 
  "RED FIVE PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(4, 4, List.of(
    new Pose2d(Constants.Field.RED_2.minus(new Translation2d(3.5, 0)), Rotation2d.fromDegrees(0)),
    new Pose2d(Constants.Field.RED_2, Rotation2d.fromDegrees(0))
  ), 
  false, 
  "RED FIVE PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_FENDER = Trajectories.generateTrajectory(4, 2, List.of(
    new Pose2d(Constants.Field.RED_2, Rotation2d.fromDegrees(0)),
    Constants.Field.RED_FENDER_1
  ), 
  true, 
  "RED FIVE PATH_TO_FENDER"
  );

  private Trajectory PATH_TO_BALL_4_5 = Trajectories.generateTrajectory(4, 4, List.of(
    Constants.Field.RED_FENDER_1,
    new Pose2d(Constants.Field.RED_4.minus(new Translation2d(0.5, Rotation2d.fromDegrees(42))), Rotation2d.fromDegrees(42))
  ),
  false,
  "RED FIVE PATH_TO_BALL_4_5"
  );

  private Trajectory PATH_TO_FENDER_2 = Trajectories.generateTrajectory(4, 2, List.of(
    new Pose2d(Constants.Field.RED_4.minus(new Translation2d(0.5, Rotation2d.fromDegrees(42))), Rotation2d.fromDegrees(42)),
    Constants.Field.RED_FENDER_1
  ),
  true,
  "RED FIVE PATH_TO_BALL_4_5"
  );

}
