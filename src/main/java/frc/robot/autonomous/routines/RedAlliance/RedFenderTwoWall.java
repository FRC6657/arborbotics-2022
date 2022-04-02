package frc.robot.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autonomous.Trajectories;
import frc.robot.autonomous.common.FireTwo;
import frc.robot.autonomous.common.IntakePath;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class RedFenderTwoWall extends SequentialCommandGroup {
  public RedFenderTwoWall(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, IntakePistonsSubsystem pistons, FlywheelSubsystem flywheel, HoodSubsystem hood, AcceleratorSubsystem accelerator) {
    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(Constants.Field.RED_FENDER_1)),
      new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons),
      drivetrain.new TrajectoryFollowerCommand(PATH_TO_FENDER),
      new FireTwo(flywheel, hood, accelerator, pistons, 1500, 10),
      new IntakePath(TAXI_PATH, drivetrain, intake, pistons)
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3, 2, List.of(
    Constants.Field.RED_FENDER_1,
    new Pose2d(Constants.Field.RED_1.minus(new Translation2d(0, 0.5)), Rotation2d.fromDegrees(90))
  ), 
  false, 
  "RED FIVE PATH_TO_BALL_2"
  );


  private Trajectory PATH_TO_FENDER = Trajectories.generateTrajectory(3, 2, List.of(
    new Pose2d(Constants.Field.RED_1, Rotation2d.fromDegrees(90)),
    Constants.Field.RED_FENDER_1
  ), 
  true, 
  "RED FIVE PATH_TO_FENDER"
  );

  private Trajectory TAXI_PATH = Trajectories.generateTrajectory(3, 2, List.of(
    Constants.Field.RED_FENDER_1,
    new Pose2d(Constants.Field.RED_2.plus(new Translation2d(0,1.25)), Rotation2d.fromDegrees(0))
  ), 
  false, 
  "RED FIVE PATH_TO_FENDER"
  );

}
