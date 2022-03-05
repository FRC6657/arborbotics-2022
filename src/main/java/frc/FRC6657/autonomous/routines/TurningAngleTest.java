package frc.FRC6657.autonomous.routines;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;

public class TurningAngleTest extends SequentialCommandGroup {
    public TurningAngleTest(
            DrivetrainSubsystem drivetrain
    ) {
        addCommands(
                new ParallelRaceGroup(
                    new WaitCommand(30),
                    drivetrain.new TurnByAngleCommand(360)
                )
        );
    }
}
