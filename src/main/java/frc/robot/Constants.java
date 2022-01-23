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
    public static final int kPickupID = 6;
    public static final int kPCMID = 7;

    //Characterization
    public static final double kS = 0.53584;
    public static final double kV = 2.2764;
    public static final double kA = 0.73118;

    //Left PID
    public static final double leftKP = 2.4109;
    public static final double leftKI = 0;
    public static final double leftKD = 0;

    //Right PID
    public static final double rightKP = 2.4109;
    public static final double rightKI = 0;
    public static final double rightKD = 0;

    //Values
    public static final double kEncoderCPR = 2048; //Encoder Counts per Rotation
    public static final double kTrackWidth = Units.inchesToMeters(21.819200); // Distance Between Sides TODO:Measure
    public static final double kGearRatio = 75.0/7.0; //Drive Gearbox Ratio
    public static final double kWheelRadius = Units.inchesToMeters(3); //Drive wheel Radius
    public static final double kEncoderCountToMeters =  (2 * Math.PI * kWheelRadius)/(kEncoderCPR*kGearRatio); //Conversion between Counts and Meters
    public static final double kMaxSpeed = 1.5; //Meters per second
    public static final double kMaxAccel = kMaxSpeed; //Meters per second per second

    //Intake Constants
    public static final double kPivotSpeed = .5;
    public static final double kPickupSpeed = 0.5;
    public static final double kLimitValue = 0.5;

    //Pivot Constants
    public static final double kExtendedPos = 4096;
    public static final double kRetractedPos = 0;

    //Pivot PID
    public static final double pivotKP = 1;
    public static final double pivotKI = 0;
    public static final double pivotKD = 0;
    public static final double pivotKF = 0;
    public static final int pivotSlotIdx = 0;
    public static final double pivotPIDLoopIdx = 0;

}
