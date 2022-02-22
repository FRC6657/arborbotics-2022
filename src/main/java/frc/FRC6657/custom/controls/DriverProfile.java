package frc.FRC6657.custom.controls;

import frc.FRC6657.Constants;

/**
 * Author Andrew Card
 */
public class DriverProfile {
    public double kMaxSpeed;
    public double kMaxTurn;
    public double kMaxTurnDegrees;
    public double kModSpeed;
    public double kModTurn;

    public int kDriveAxis;
    public int kTurnAxis;

    public DriverProfile(
        double kMaxSpeed,
        double kMaxTurn,
        double kModSpeed,
        double kModTurn
    ){
        this.kMaxTurnDegrees = kMaxTurn;
        this.kMaxSpeed = kMaxSpeed;
        this.kMaxTurn = (Math.PI*Constants.Drivetrain.kTrackWidth)/(360/kMaxTurn);
        this.kModSpeed = kModSpeed;
        this.kModTurn = (Math.PI*Constants.Drivetrain.kTrackWidth)/(360/kModTurn);
    }

}
