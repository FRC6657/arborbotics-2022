package frc.robot.autonomous.routines.BlueAllience;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autonomous.Trajectories;
import frc.robot.autonomous.common.FireOne;
import frc.robot.autonomous.common.FireTwo;
import frc.robot.autonomous.common.IntakePath;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class BlueFenderThree extends SequentialCommandGroup {
  public BlueFenderThree(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, IntakePistonsSubsystem pistons, AcceleratorSubsystem accelerator, FlywheelSubsystem flywheel, HoodSubsystem hood) {
    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(Constants.Field.BLUE_FENDER_1)),
      new FireOne(flywheel, hood, accelerator, pistons, 2500, 1),
      new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons),
      new IntakePath(PATH_TO_BALL_3, drivetrain, intake, pistons),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_FENDER),
      new FireTwo(flywheel, hood, accelerator, pistons, 2500, 1)
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3, 2, List.of(
    Constants.Field.BLUE_FENDER_1,
    new Pose2d(Constants.Field.BLUE_1.plus(new Translation2d(0, 0.5)), Rotation2d.fromDegrees(-90))
  ), 
  false, 
  "BLUE FIVE PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_BALL_3 = Trajectories.generateTrajectory(3, 2, List.of(
    new Pose2d(Constants.Field.BLUE_2.plus(new Translation2d(3.5, 0)), Rotation2d.fromDegrees(180)),
    new Pose2d(Constants.Field.BLUE_2, Rotation2d.fromDegrees(180))
  ), 
  false, 
  "BLUE FIVE PATH_TO_BALL_2"
  );

  private Trajectory PATH_TO_FENDER = Trajectories.generateTrajectory(3, 2, List.of(
    new Pose2d(Constants.Field.BLUE_2, Rotation2d.fromDegrees(180)),
    Constants.Field.BLUE_FENDER_1
  ), 
  true, 
  "BLUE FIVE PATH_TO_FENDER"
  );

}
