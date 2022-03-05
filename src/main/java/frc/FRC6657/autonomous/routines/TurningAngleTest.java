package frc.FRC6657.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.FRC6657.subsystems.drivetrain.DrivetrainSubsystem;

public class TurningAngleTest extends SequentialCommandGroup{
    public TurningAngleTest(
        DrivetrainSubsystem drivetrain
    ) {
        addCommands(drivetrain.new TurnByAngleCommand(360));
    }
}
