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

    //Characterization
    public static final double kS = 0.53584;
    public static final double kV = 2.2764;
    public static final double kA = 0.73118;
    public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(kS, kV, kA);

    //PID
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final PIDController kDrivePIDController = new PIDController(kP, kI, kD);

    //Values
    public static final double kRobotWeight = Units.lbsToKilograms(40);
    public static final double kEncoderCPR = 2048; //Encoder Counts per Rotation
    public static final double kTrackWidth = Units.inchesToMeters(21.819200); // Distance Between Sides TODO:Measure
    public static final double kGearRatio = 10.71; //Drive Gearbox Ratio
    public static final double kWheelRadius = Units.inchesToMeters(3); //Drive wheel Radius
    public static final double kDistancePerPulse =  (2 * Math.PI * kWheelRadius / kEncoderCPR); //Conversion between Counts and Meters
    public static final double kMaxSpeed = 5; //Meters/s

    //Default Sim
    public static final DifferentialDrivetrainSim kDrivetrainSim = new DifferentialDrivetrainSim( //Simulation
        DCMotor.getFalcon500(2),
        kGearRatio,
        7.5,
        kRobotWeight,
        kWheelRadius,
        kTrackWidth,
        null);

}
