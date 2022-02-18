package frc.FRC6657.custom.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.ctre.IdleMode;

public class DriverProfile {
    public double kMaxSpeed;
    public double kMaxTurn;
    public double kMaxTurnDegrees;
    public IdleMode kIdleMode;
    public double kModSpeed;
    public double kModTurn;

    public int kDriveAxis;
    public int kTurnAxis;

    public DriverProfile(
        double kMaxSpeed,
        double kMaxTurn,
        double kModSpeed,
        double kModTurn,
        IdleMode kIdleMode
    ){
        this.kMaxTurnDegrees = kMaxTurn;
        this.kMaxSpeed = kMaxSpeed;
        this.kMaxTurn = (Math.PI*Constants.Drivetrain.kTrackWidth)/(360/kMaxTurn);
        this.kIdleMode = kIdleMode;
        this.kModSpeed = kModSpeed;
        this.kModTurn = (Math.PI*Constants.Drivetrain.kTrackWidth)/(360/kModTurn);
    }

}
