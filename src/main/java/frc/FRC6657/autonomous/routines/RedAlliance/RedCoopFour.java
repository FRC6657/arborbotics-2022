package frc.FRC6657.autonomous.routines.RedAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.autonomous.common.AimRoutine;
import frc.FRC6657.autonomous.common.FireRoutine;
import frc.FRC6657.autonomous.common.IntakePath;
import frc.FRC6657.autonomous.common.ShootPath;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class RedCoopFour extends SequentialCommandGroup{
    public RedCoopFour(
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem pistons,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator,
        HoodSubsystem hood,
        VisionSupplier vision
    ) {
        addCommands(
            new InstantCommand(() -> drivetrain.resetPoseEstimator(PATH_TO_BALL_2.getInitialPose()), drivetrain), //Reset Position
            new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons), //Intake Red 2
            new AimRoutine(drivetrain, hood, flywheel, vision), //Aim
            new FireRoutine(flywheel, hood, accelerator, 0.5), //Fire Red 1 & 2
            new IntakePath(PATH_TO_BALL_3_4, drivetrain, intake, pistons),
            new ShootPath(PATH_TO_3_4_SHOT, drivetrain, hood, flywheel, vision),
            new FireRoutine(flywheel, hood, accelerator, 0.5)
        );
    }

    private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(2,4,List.of(
        new Pose2d(9.802, 5.55, Rotation2d.fromDegrees(24)),
        new Pose2d(11.218, 6.353,Rotation2d.fromDegrees(24))
    ),
    false,
    "Red Mid Two PATH_TO_BALL_2"
    );

    private Trajectory PATH_TO_BALL_3_4 = Trajectories.generateTrajectory(3,2,List.of(
        new Pose2d(12.75, 5, Rotation2d.fromDegrees(38)),
        new Pose2d(14.868, 6.61,Rotation2d.fromDegrees(38))
    ),
    false,
    "Red Mid Two PATH_TO_BALL_2"
    );

    private Trajectory PATH_TO_3_4_SHOT = Trajectories.generateTrajectory(3,2,List.of(
        new Pose2d(14.868, 6.61,Rotation2d.fromDegrees(38)),
        new Pose2d(11.75, 5.895,Rotation2d.fromDegrees(30))
    ),
    true,
    "Red Mid Two PATH_TO_BALL_2"
    );
}
