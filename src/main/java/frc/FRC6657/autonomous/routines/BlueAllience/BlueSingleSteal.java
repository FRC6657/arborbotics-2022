package frc.FRC6657.autonomous.routines.BlueAllience;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.autonomous.common.AimRoutine;
import frc.FRC6657.autonomous.common.FireRoutine;
import frc.FRC6657.autonomous.common.IntakePath;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class BlueSingleSteal extends SequentialCommandGroup{
    public BlueSingleSteal(
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem pistons,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator,
        HoodSubsystem hood,
        VisionSupplier vision
    ) {
        addCommands(
            new InstantCommand(() -> drivetrain.resetPoseEstimator(PATH_TO_BLUE_2.getInitialPose()), drivetrain),
            new IntakePath(PATH_TO_BLUE_2, drivetrain, intake, pistons),
            new AimRoutine(drivetrain, hood, flywheel, vision),
            new FireRoutine(flywheel, hood, accelerator, 0.5),
            new IntakePath(PATH_TO_RED_1, drivetrain, intake, pistons),
            drivetrain.new TrajectoryFollowerCommand(PATH_TO_RED_SHOT_1),
            new ParallelCommandGroup(
                new InstantCommand(() -> hood.setAngle(45), hood),
                new InstantCommand(() -> flywheel.setRPMTarget(1000), flywheel)
            ),
            new FireRoutine(flywheel, hood, accelerator, 0.5)
        );
    }

    private Trajectory PATH_TO_BLUE_2 = Trajectories.generateTrajectory(4, 4, List.of(
        new Pose2d(6.140, 5.179, Rotation2d.fromDegrees(-220)),
        new Pose2d(5.014, 6.16, Rotation2d.fromDegrees(-220))

    ), false, 
    "Blue Single Steal 2 PATH_TO_BLUE_2"
    );

    private Trajectory PATH_TO_RED_1 = Trajectories.generateTrajectory(1, 2,List.of(
        new Pose2d(6.766, 4.661, Rotation2d.fromDegrees(155.98)),
        new Pose2d(5.9, 7.082, Rotation2d.fromDegrees(96))

    ), 
    false, 
    "Blue Single Steal PATH_TO_RED_1"
    );

    private Trajectory PATH_TO_RED_SHOT_1 = Trajectories.generateTrajectory(1, 2,List.of(
        new Pose2d(6, 6.5, Rotation2d.fromDegrees(0)),
        new Pose2d(5, 6.5, Rotation2d.fromDegrees(0))

    ), 
    true, 
    "Blue Single Steal PATH_TO_RED_SHOT_1"
    );

}

