package frc.FRC6657.autonomous.routines.BlueAlliance;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.FRC6657.autonomous.Trajectories;
import frc.FRC6657.custom.ArborSequentialCommandGroup;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;
import frc.FRC6657.subsystems.intake.ExtensionSubsystem;
import frc.FRC6657.subsystems.intake.IntakeSubsystem;
import frc.FRC6657.subsystems.shooter.AcceleratorSubsystem;
import frc.FRC6657.subsystems.shooter.FlywheelSubsystem;
import frc.FRC6657.subsystems.shooter.HoodSubsystem;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

public class BlueDoubleSteal extends ArborSequentialCommandGroup{
    public BlueDoubleSteal(
        DrivetrainSubsystem drivetrain,
        IntakeSubsystem intake,
        ExtensionSubsystem pistons,
        FlywheelSubsystem flywheel,
        AcceleratorSubsystem accelerator,
        HoodSubsystem hood,
        VisionSupplier vision
    ) {
        addReqs(drivetrain, intake, pistons, flywheel, hood, accelerator, vision);
        addCommands(
            new ParallelRaceGroup(
                new WaitUntilCommand(intake::ballDetected),
                drivetrain.new TrajectoryFollowerCommand(PATH_TO_BALL_1, true)
            )
            .beforeStarting(
                new ParallelCommandGroup(
                    new InstantCommand(pistons::extend),
                    new InstantCommand(intake::start)
                )
            )
            .andThen(
                new ParallelCommandGroup(
                    new InstantCommand(pistons::retract),
                    new InstantCommand(intake::stop)
                )
            ),
            new TurnAndShoot(drivetrain.new TrajectoryFollowerCommand(PATH_TO_SHOOT, false)),
            new ParallelRaceGroup(
                new WaitUntilCommand(intake::ballDetected),
                drivetrain.new TrajectoryFollowerCommand(PATH_TO_RED_1, false)
            )
            .beforeStarting(
                new ParallelCommandGroup(
                    new InstantCommand(pistons::extend),
                    new InstantCommand(intake::start)
                )
            )
            .andThen(
                new ParallelCommandGroup(
                    new InstantCommand(pistons::retract),
                    new InstantCommand(intake::stop)
                )  
            ),
            new ParallelRaceGroup(
                drivetrain.new TrajectoryFollowerCommand(PATH_TO_RED_2, false),
                new RunCommand(() -> flywheel.setRPMTarget(2000)),
                new RunCommand(
                    () -> {
                        hood.setAngle(45);
                        System.out.println(vision.getDistance());
                    }, 
                    hood
                )
            )//Get rid of our stolen goods
            .andThen(
                new SequentialCommandGroup( //TODO This seems coarse, fix later.
                    new WaitUntilCommand(flywheel::atTarget),
                    new InstantCommand(accelerator::start)
                ).andThen(
                    new ParallelCommandGroup(
                        new InstantCommand(accelerator::stop),
                        new InstantCommand(flywheel::stop),
                        hood.new Home()
                    )
                )
            )
        );
    }

    private Trajectory PATH_TO_BALL_1 = Trajectories.generateTrajectory(4, 4, List.of(
        new Pose2d(6.140, 5.179, Rotation2d.fromDegrees(-220)),
        new Pose2d(5.014, 6.16, Rotation2d.fromDegrees(-220))

    ), false, 
    "Blue Steal 2 PATH_TO_BALL"
    );

    private Trajectory PATH_TO_SHOOT = Trajectories.generateTrajectory(1,2,List.of(
        new Pose2d(5.014, 6.16, Rotation2d.fromDegrees(-220)),
        new Pose2d(6.766, 4.661, Rotation2d.fromDegrees(155.98))
    ),
    true,
    "Blue Steal PATH_TO_SHOOT"
    );

    private Trajectory PATH_TO_RED_1 = Trajectories.generateTrajectory(1, 2,List.of(
        new Pose2d(6.766, 4.661, Rotation2d.fromDegrees(155.98)),
        new Pose2d(6.122, 7.082, Rotation2d.fromDegrees(96))

    ), 
    false, 
    "Blue Steal PATH_TO_RED_1"
    );

    private Trajectory PATH_TO_RED_2 = Trajectories.generateTrajectory(4, 4, List.of(
        new Pose2d(6.122, 7.082, Rotation2d.fromDegrees(-135)),
        new Pose2d(4.467, 3.620, Rotation2d.fromDegrees(-71.657))
    ), 
    false, 
    "Blue Steal Two PATH_TO_RED_2"
    );


}

