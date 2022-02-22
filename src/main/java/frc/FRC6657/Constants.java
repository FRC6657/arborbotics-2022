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

import frc.FRC6657.custom.rev.Blinkin.BlinkinLEDPattern;

public final class Constants {

    // Drivetrain //

    //PWM IDs
    public static final int kBlinkinID = 0;

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
    public static final int kHoodID = 11;
    public static final int kRightLiftID = 12; 
    public static final int kLeftLiftID = 13;


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
        public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(drive_kS, drive_kV, drive_kA);

        // Drivetrain PID
        public static final double drive_linear_kP = 1; //Char P gain 3.1976

        public static final PIDController kLinearPIDController = new PIDController(drive_linear_kP, 0, 0);

        // Drivetrain Values
        public static final double kRobotWeight = Units.lbsToKilograms(40);
        public static final double kTrackWidth = Units.inchesToMeters(21.819200); // Distance Between Sides
        public static final double kGearRatio = 75 / 7; // Drive Gearbox Ratio
        public static final double kWheelRadius = Units.inchesToMeters(3); // Drive wheel Radius
        public static final double kDistancePerPulse = (2 * Math.PI * kWheelRadius) / (kFalconEncoderCPR * kGearRatio); // Conversion between Counts and Meters
        public static final double kAimTollerance = 3;
        public static final double kDistanceTollerance = 0.1;


        //TODO Put robot on cart and figure out these values.
        public static final double kMaxAttainableSpeed = 5;
        public static final double kMaxAttainableTurnRate = Units.radiansToDegrees(kMaxAttainableSpeed * kTrackWidth/2);

        // Default Sim
        public static final DifferentialDrivetrainSim kSim = new DifferentialDrivetrainSim( // Simulation
            DCMotor.getFalcon500(2),
            kGearRatio,
            7.5,
            kRobotWeight,
            kWheelRadius,
            kTrackWidth,
            null
        );
    }

    /**
     * Intake Values
     */
    public class Intake {
        public static final double kSpeed = 1;

        public static final double kStartupTime = 0.1; //Sec
        public static final double kBallCurrent = 20; //Amps

    }

    public class Lift {
        public static final double encoderCPR = 8192;
        public static final double kP = 1;
        public static final int rightEncoderID = 1;
        public static final int leftEncoderID = 2;

    }

    /**
     * Shooter Values
     */

    public static class Flywheel {
        // General
        public static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500);
        public static final double kRPMTollerance = 50;
        public static final double kRatio = 1.5/1.0;

        // Characterization
        public static final double kV = 0.0549; // 0.051151
        public static final double kA= 0.0024487; //0.0024487 

        public static final LinearSystem<N1, N1, N1> kPlant = LinearSystemId.identifyVelocitySystem(kV, kA);

        public static final FlywheelSim kSim = new FlywheelSim(
            kPlant,
            DCMotor.getFalcon500(1),
            kRatio
        );

    }

    public static class Hood {
        public static final double kUpSpeed = 0.5;
        public static final double kDownSpeed = -0.3;
    }

    
    /**
     * Accelerator Values
     */
    public static class Accelerator{
        public static final double kSpeed = 1;
    }

    public static class Vision{
        public static final double kCameraHeightMeters = Units.inchesToMeters(35.111986);// CAD Estimate
        public static final double kTargetHeightMeters = Units.feetToMeters(8 + 8 / 12); // 8` 8" from manual
        public static final double kCameraPitchRadians = Units.degreesToRadians(30); // CAD Estimate
        public static final double kCamDiagFOV = 67.8; // degrees
        public static final int kCamResolutionWidth = 320; // pixels
        public static final int kCamResolutionHeight = 240; // pixels
        public static final double kTargetWidth = Units.feetToMeters(4);  
    }

    /**
     * Pretty Lights
     */
    public static class BlinkinColors{
        public static final BlinkinLEDPattern kIdle = BlinkinLEDPattern.COLOR_WAVES_FOREST_PALETTE;
        public static final BlinkinLEDPattern kIntake = BlinkinLEDPattern.SOLID_WHITE;
        public static final BlinkinLEDPattern kBallDetected = BlinkinLEDPattern.SOLID_LIME;
        public static final BlinkinLEDPattern kReadyFlywheel = BlinkinLEDPattern.RAINBOW_PALETTE;
        public static final BlinkinLEDPattern kNotReadyFlywheel = BlinkinLEDPattern.STROBE_RED;
    }

    public static class DriverConfigs{
        public static final double kTurnDeadband = 0.1;
        public static final double kDriveDeadband = 0.1;
    }
    
}
