// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.Interpolation;

// Shot parameter
public class ShotParameter {

    // Variables
    public final double rpm;
    public final double hoodAngle;

    // Constructor
    public ShotParameter(double hoodAngle, double rpm) {
        this.rpm = rpm;
        this.hoodAngle = hoodAngle;
    }   

    // Method equals
    public boolean equals(ShotParameter other) {
        return Math.abs(this.hoodAngle - other.hoodAngle) < 0.1 &&
        Math.abs(this.rpm - other.rpm) < 0.1;
    }

    // Method to interpolate
    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(hoodAngle, end.hoodAngle, t),
            lerp(rpm, end.rpm, t)
        );
    }

    // Linear Interpolation method
    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }
 
}