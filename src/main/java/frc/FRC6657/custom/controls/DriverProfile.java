package frc.FRC6657.custom.controls;

import frc.FRC6657.Constants;
import frc.FRC6657.custom.ctre.NeutralMode;

public class DriverProfile {
    public double kMaxSpeed;
    public double kMaxTurn;
    public NeutralMode kIdleMode;
    public DriverProfile(double kMaxSpeed, double kMaxTurn, NeutralMode kIdleMode){
        this.kMaxSpeed = kMaxSpeed;
        this.kMaxTurn = (Math.PI*Constants.Drivetrain.kTrackWidth)/(360/kMaxTurn);
        this.kIdleMode = kIdleMode;
    }
}
