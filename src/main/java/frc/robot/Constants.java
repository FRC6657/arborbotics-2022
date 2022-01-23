// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

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


    //Drivetrain Characterization
    public static final double drive_kS = 0.53584;
    public static final double drive_kV = 2.2764;
    public static final double drive_kA = 0.73118;
    public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(drive_kS, drive_kV, drive_kA);

    //Drivetrain PID
    public static final double drive_kP = 3.1976; 
    public static final double drive_kI = 0;
    public static final double drive_kD = 0;
    public static final PIDController kDrivePIDController = new PIDController(drive_kP, drive_kI, drive_kD);

    //Drivetrain Values
    public static final double kRobotWeight = Units.lbsToKilograms(40);
    public static final double kFalconEncoderCPR = 2048; //Encoder Counts per Rotation
    public static final double kTrackWidth = Units.inchesToMeters(21.819200); // Distance Between Sides
    public static final double kGearRatio = 10.71; //Drive Gearbox Ratio
    public static final double kWheelRadius = Units.inchesToMeters(3); //Drive wheel Radius
    public static final double kDistancePerPulse =  (2 * Math.PI * kWheelRadius / kFalconEncoderCPR); //Conversion between Counts and Meters
    public static final double kMaxSpeed = 3.5; //Meters/s

        //Default Sim
    public static final DifferentialDrivetrainSim kDrivetrainSim = new DifferentialDrivetrainSim( //Simulation
        DCMotor.getFalcon500(2),
        kGearRatio,
        7.5,
        kRobotWeight,
        kWheelRadius,
        kTrackWidth,
        null);
  
    public static final double kEncoderCountToMeters =  (2 * Math.PI * kWheelRadius)/(kEncoderCPR*kGearRatio); //Conversion between Counts and Meters
    public static final double kMaxSpeed = 1.5; //Meters per second
    public static final double kMaxAccel = kMaxSpeed; //Meters per second per second

    //Intake Constants
    public static final double kPivotSpeed = .5;
    public static final double kPickupSpeed = 0.5;
    public static final double kLimitValue = 0.5;

}
