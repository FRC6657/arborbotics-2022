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

public class Taxi extends SequentialCommandGroup {
  public Taxi(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, IntakePistonsSubsystem pistons, AcceleratorSubsystem accelerator, FlywheelSubsystem flywheel, HoodSubsystem hood, VisionSupplier vision) {
    addCommands(
      drivetrain.new TeleOpCommand(() -> 0.5,() -> 0,() -> false).withTimeout(1),
      new InstantCommand(drivetrain::stop),
      new FireTwo(flywheel, hood, accelerator, pistons, InterpolatingTable.get(vision.getYaw()).rpm, InterpolatingTable.get(vision.getYaw()).rpm)
    );
  }

}
