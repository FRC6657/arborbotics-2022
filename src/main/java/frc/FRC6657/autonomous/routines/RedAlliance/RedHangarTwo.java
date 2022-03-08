package frc.FRC6657.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

public class RedHangarTwo extends SequentialCommandGroup{
    public RedHangarTwo (
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem pistons,
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        AcceleratorSubsystem accelerator,
        VisionSupplier vision
    ) {
        addCommands(
            drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_1, true),
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
            )
        );
    }

    private Trajectory PATH_TO_BALL_1 = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(9.7, 2.633918, Rotation2d.fromDegrees(-66.25)),
        new Pose2d(11.58, 2, Rotation2d.fromDegrees(-30))

    ), false, 
    "Red Hangar 2 PATH_TO_BALL"
    );
    
}
