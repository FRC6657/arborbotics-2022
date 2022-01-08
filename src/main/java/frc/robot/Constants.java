// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    
    // Drivetrain //

    //CAN IDs
    public static final int kFrontLeftID = 1;
    public static final int kFrontRightID = 2;
    public static final int kBackLeftID = 3;
    public static final int kBackRightID = 4;
    public static final int kPigeonID = 5;
    public static final int kIntakeID = 6;

    //Characterization
    public static final double kS = 1;
    public static final double kV = 3;

    //Left PID
    public static final double leftKP = 1;
    public static final double leftKI = 0;
    public static final double leftKD = 0;

    //Right PID
    public static final double rightKP = 1;
    public static final double rightKI = 0;
    public static final double rightKD = 0;

    //Values
    public static final double kEncoderCPR = 2048; //Encoder Counts per Rotation
    public static final double kTrackWidth = Units.inchesToMeters(26); // Distance Between Sides TODO:Measure
    public static final double kGearRatio = 75.0/7.0; //Drive Gearbox Ratio
    public static final double kWheelRadius = Units.inchesToMeters(3); //Drive wheel Radius
    public static final double kEncoderCountToMeters =  (2 * Math.PI * kWheelRadius)/(kEncoderCPR*kGearRatio); //Conversion between Counts and Meters
    public static final double kMaxSpeed = 3.5; //3.5 Meters/s

}
