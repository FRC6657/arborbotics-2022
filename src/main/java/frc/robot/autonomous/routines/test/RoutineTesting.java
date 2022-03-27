package frc.robot.autonomous.routines.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.autonomous.Trajectories;
import frc.robot.autonomous.common.IntakePath;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import frc.robot.subsystems.intake.IntakePistonsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class RoutineTesting extends SequentialCommandGroup {
  public RoutineTesting(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, IntakePistonsSubsystem pistons, FlywheelSubsystem flywheel, HoodSubsystem hood,AcceleratorSubsystem accelerator ,VisionSupplier vision) {
    addCommands(
      new InstantCommand(() -> drivetrain.resetOdometry(Trajectories.Test.getInitialPose()), drivetrain),
      new IntakePath(Trajectories.Test, drivetrain, intake, pistons),
      new ParallelCommandGroup(
        drivetrain.new VisionAimAssist()
        .beforeStarting(
          new SequentialCommandGroup(
            new InstantCommand(() -> vision.enableLEDs()),
            new WaitCommand(0.25)
          )
          ).andThen(new InstantCommand(() -> vision.disableLEDs())),
          new SequentialCommandGroup(
        new WaitUntilCommand(() -> (hood.ready() && flywheel.ready())),
        new ParallelCommandGroup(
          new StartEndCommand(accelerator::start, accelerator::stop, accelerator),
          new StartEndCommand(pistons::extend, pistons::retract, pistons)
        )
      )
      )
    );
  }
}
