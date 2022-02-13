package frc.FRC6657.custom.controls;

import frc.FRC6657.Constants;

public class DriverProfile {
    public double kMaxSpeed;
    public double kMaxTurn;
    public DriverProfile(double kMaxSpeed, double kMaxTurn){
        this.kMaxSpeed = kMaxSpeed;
        this.kMaxTurn = (Math.PI*Constants.Drivetrain.kTrackWidth)/(360/kMaxTurn);
    }
}
