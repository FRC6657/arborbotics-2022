package frc.FRC6657.autonomous.routines.BlueAllience;

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

public class BlueCoopFour extends SequentialCommandGroup{
    public BlueCoopFour(
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
          new IntakePath(PATH_TO_BALL_2, drivetrain, intake, pistons), //Intake Blue 2
          new AimRoutine(drivetrain, hood, flywheel, vision), //Aim
          new FireRoutine(flywheel, hood, accelerator, 0.5), //Fire Blue 1 & 2
          new IntakePath(PATH_TO_BALL_3_4, drivetrain, intake, pistons),
          new ShootPath(PATH_TO_3_4_SHOT, drivetrain, hood, flywheel, vision),
          new FireRoutine(flywheel, hood, accelerator, 0.5)
        );
    }

    private Trajectory PATH_TO_BALL_2 = Trajectories.generateTrajectory(4,1,List.of(
        new Pose2d(6.666, 2.737, Rotation2d.fromDegrees(200)),
        new Pose2d(5.164,2.044,Rotation2d.fromDegrees(200))
    ),
    false,
    "Blue Mid Two PATH_TO_BALL_2"
    );

    private Trajectory PATH_TO_BALL_3_4 = Trajectories.generateTrajectory(4,2,List.of(
        new Pose2d(3.863, 3.5, Rotation2d.fromDegrees(-138)),
        new Pose2d(1.65, 1.7,Rotation2d.fromDegrees(-135))
    ),
    false,
    "Blue Mid Two PATH_TO_BALL_2"
    );

    private Trajectory PATH_TO_3_4_SHOT = Trajectories.generateTrajectory(4,2,List.of(
        new Pose2d(1.65, 1.7,Rotation2d.fromDegrees(-135)),
        new Pose2d(4.75, 2.5,Rotation2d.fromDegrees(-153))
    ),
    true,
    "Blue Mid Two PATH_TO_BALL_2"
    );

}