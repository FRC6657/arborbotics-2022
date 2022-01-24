// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public final class Constants {

    // Drivetrain //

    // CAN IDs
    public static final int kFrontLeftID = 1;
    public static final int kFrontRightID = 2;
    public static final int kBackLeftID = 3;
    public static final int kBackRightID = 4;
    public static final int kPigeonID = 5;
    public static final int kPickupID = 6;
    public static final int kPCMID = 7;
    public static final int kLeftFlywheelID = 8;
    public static final int kRightFlywheelID = 9;
    public static final int kAcceleratorID = 10;

    // General Values
    public static final double kFalconEncoderCPR = 2048; // Encoder Counts per Rotation

    /**
     * Drivetrain Values
     */
    public static class Drivetrain {
        // Drivetrain Characterization
        public static final double drive_kS = 0.53584;
        public static final double drive_kV = 2.2764;
        public static final double drive_kA = 0.73118;
        public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(drive_kS, drive_kV,
                drive_kA);

        // Drivetrain PID
        public static final double drive_kP = 3.1976;
        public static final double drive_kI = 0;
        public static final double drive_kD = 0;
        public static final PIDController kDrivePIDController = new PIDController(drive_kP, drive_kI, drive_kD);

        // Drivetrain Values
        public static final double kRobotWeight = Units.lbsToKilograms(40);
        public static final double kTrackWidth = Units.inchesToMeters(21.819200); // Distance Between Sides
        public static final double kGearRatio = 75 / 7; // Drive Gearbox Ratio
        public static final double kWheelRadius = Units.inchesToMeters(3); // Drive wheel Radius
        public static final double kDistancePerPulse = (2 * Math.PI * kWheelRadius) / (kFalconEncoderCPR * kGearRatio); // Conversion
                                                                                                                        // between
                                                                                                                        // Counts
                                                                                                                        // and
                                                                                                                        // Meters
        public static final double kMaxSpeed = 3.5; // Meters per second
        public static final double kMaxAccel = kMaxSpeed * 3; // Meters per second per second

        // Default Sim
        public static final DifferentialDrivetrainSim kSim = new DifferentialDrivetrainSim( // Simulation
                DCMotor.getFalcon500(2),
                kGearRatio,
                7.5,
                kRobotWeight,
                kWheelRadius,
                kTrackWidth,
                null);
    }

    /**
     * Intake Values
     */
    public class Intake {
        public static final double kSpeed = 0.55;
    }

    /**
     * Shooter Values
     */

    public static class Flywheel {
        // General
        public static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500);
        public static final double kRPMTollerance = 50;
        public static final double kRatio = 1.0/2.0;

        // Characterization
        public static final double kV = 0.051151;
        public static final double kA= 0.0024487;

        public static final LinearSystem<N1, N1, N1> kPlant = LinearSystemId.identifyVelocitySystem(kV, kA);

        public static final FlywheelSim kSim = new FlywheelSim(
            kPlant,
            DCMotor.getFalcon500(1),
            0.5
        );

    }

    
    /**
     * Accelerator Values
     */
    public static class Accelerator{
        public static final double kSpeed = 1;
    }

}