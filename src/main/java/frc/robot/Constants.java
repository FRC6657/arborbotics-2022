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

    //Characterization
    public static final double kS = 0;
    public static final double kV = 0;

    //Measurements
    public static final double kEncoderCPR = 2048;
    public static final double kTrackWidth = Units.inchesToMeters(26);
    public static final double kGearRatio = 75.0/7.0;
    public static final double kWheelRadius = Units.inchesToMeters(3);
    public static final double kEncoderCountToMeters =  (2 * Math.PI * kWheelRadius)/(kEncoderCPR*kGearRatio);

}
