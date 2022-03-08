package frc.FRC6657.autonomous.routines.BlueAllience;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;

public class BlueHangarTwo extends SequentialCommandGroup{
    public BlueHangarTwo (
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem pistons,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator
    ) {
        addCommands(
        );
    }

    private Trajectory PATH_TO_BALL_1 = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(9.7, 2.633918, Rotation2d.fromDegrees(-66.25)),
        new Pose2d(11.58, 2, Rotation2d.fromDegrees(-30))

    ), false, 
    "Blue Hangar 2 PATH_TO_BALL"
    );

    private Trajectory PATH_TO_SHOOT = Trajectories.generateTrajectory(1,2,List.of(
        new Pose2d(5.014, 6.16, Rotation2d.fromDegrees(-220)),
        new Pose2d(6.766, 4.661, Rotation2d.fromDegrees(155.98))
    ),
    true,
    "Blue Hangar TWO PATH_TO_SHOOT"
    );

    private Trajectory PATH_TO_PARK = Trajectories.generateTrajectory(1, 2, List.of(
        new Pose2d(6.766, 4.661, Rotation2d.fromDegrees(155.98)),
        new Pose2d(5.014, 6.16, Rotation2d.fromDegrees(-223))
    ), 
    false, 
    "Blue Hangar TWO PATH_TO_PARK"
    );
    
}
