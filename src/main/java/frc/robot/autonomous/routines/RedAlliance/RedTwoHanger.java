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
import frc.robot.autonomous.common.FireOne;
import frc.robot.autonomous.common.FireTwo;
import frc.robot.autonomous.common.IntakePath;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.Interpolation.InterpolatingTable;

public class RedTwoHanger extends SequentialCommandGroup {
  public RedTwoHanger(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, IntakePistonsSubsystem pistons, FlywheelSubsystem flywheel, HoodSubsystem hood, AcceleratorSubsystem accelerator, VisionSupplier vision) {
    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(9.871, 2.470, Rotation2d.fromDegrees(48)))),
      new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons),
      new FireTwo(flywheel, hood, accelerator, pistons, InterpolatingTable.get(vision.getPitch()).rpm, InterpolatingTable.get(vision.getPitch()).hoodAngle)
    );
  }

  private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(3, 2, List.of(
    new Pose2d(9.871, 2.470, Rotation2d.fromDegrees(48)),
    new Pose2d(11.647, 1.992, Rotation2d.fromDegrees(-25))
  ), 
  false, 
  "RED FIVE PATH_TO_BALL_2"
  );
}
