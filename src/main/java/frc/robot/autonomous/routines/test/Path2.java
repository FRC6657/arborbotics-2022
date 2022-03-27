package frc.robot.autonomous.routines.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.Trajectories;
import frc.robot.autonomous.common.IntakePath;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class Path2 extends SequentialCommandGroup {
  public Path2(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, IntakePistonsSubsystem pistons, FlywheelSubsystem flywheel, HoodSubsystem hood,AcceleratorSubsystem accelerator ,VisionSupplier vision) {
    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.Test2.getInitialPose()), drivetrain),
      new IntakePath(Trajectories.Test2, drivetrain, intake, pistons)
    );
  }
}
