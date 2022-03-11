// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657;

import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
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
        public static final double linear_kS = 0.53584;
        public static final double linear_kV = 2.2764;
        public static final double linear_kA = 0.73118;
        //public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(linear_kS, linear_kV, linear_kA);


        public static final double angular_kS = 0.1;
        public static final double angular_kV = 0.1;
        public static final double angular_kA = 0.1;

        // Drivetrain PID
        public static final double drive_linear_kP = 0.64132; //Char P gain 0.64132
        public static final double Turn_Command_kP = 1d/90;
        public static final double Turn_Command_kD = 1d/500;

        public static final PIDController kLinearPIDController = new PIDController(drive_linear_kP, 0, 0);

        public static final double vision_kP = 6d/27;
        public static final double vision_kD = 1d/30;

        // Drivetrain Values
        public static final double kRobotWeight = Units.lbsToKilograms(40);
        public static final double kTrackWidth = Units.inchesToMeters(21.819200); // Distance Between Sides
        public static final double kGearRatio = 75d / 7; // Drive Gearbox Ratio
        public static final double kWheelRadius = Units.inchesToMeters(3); // Drive wheel Radius
        public static final double kDistancePerPulse = (2 * Math.PI * kWheelRadius) / (kFalconEncoderCPR * kGearRatio); // Conversion between Counts and Meters
        public static final double kAimTollerance = 3;
        public static final double kDistanceTollerance = 0.1;
        public static final double kTurnCommandTolerance = 0.5;
      
        public static final double kMaxAttainableSpeed = ((6380d/60) * (6 * Math.PI))/(39.37*kGearRatio);
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

        public static final LinearSystem<N2,N2,N2> kPlant = LinearSystemId.identifyDrivetrainSystem(
            linear_kV,
            linear_kA,
            angular_kV,
            angular_kA
        );

        public static final LinearPlantInversionFeedforward<N2,N2,N2> kFeedForward = new LinearPlantInversionFeedforward<N2,N2,N2>(kPlant,0.02);

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

        public static final double kSpeed = 0.5;

        // Characterization
        public static final double kV = 0.0549/4.5; // 0.051151
        public static final double kA= 0.0024487/4.5; //0.0024487 

        public static final LinearSystem<N1, N1, N1> kPlant = LinearSystemId.identifyVelocitySystem(kV, kA);

        public static final FlywheelSim kSim = new FlywheelSim(
            kPlant,
            DCMotor.getFalcon500(2),
            kRatio
        );

    }

    public static class Hood {
        public static final double kUpSpeed = 0.5;
        public static final double kDownSpeed = -0.3;
        public static final double kRatio = 1d/(((10*5*3)*(3/2d)*(5/2d)));
        public static final double kP = 12/45d;
        public static final PIDController kPIDController = new PIDController(kP, 0, 0);

    }

    
    /**
     * Accelerator Values
     */
    public static class Accelerator{
        public static final double kSpeed = 1;
    }

    public static class Vision{
        public static final String kCameraName = "limelight";
        public static final double kCameraHeightMeters = 0.638374;// CAD Estimate
        public static final double kTargetHeightMeters = Units.feetToMeters(8 + 8 / 12); // 8` 8" from manual
        public static final double kCameraPitchRadians = Units.degreesToRadians(42);
        public static final double kMaxLEDRange = 20;
        public static final double kCamDiagFOV = 67.8; // degrees
        public static final int kCamResolutionWidth = 320; // pixels
        public static final int kCamResolutionHeight = 240; // pixels
        public static final double kTargetWidth = Units.feetToMeters(4);  
        public static final double kMinTargetArea = 10;
        public static final Pose2d kTargetPos1 = new Pose2d(8.259, 4.138, Rotation2d.fromDegrees(0));
        public static final Pose2d kTargetPos2 = new Pose2d(8.259, 4.138, Rotation2d.fromDegrees(45));
        public static final Pose2d kTargetPos3 = new Pose2d(8.259, 4.138, Rotation2d.fromDegrees(90));
        public static final Pose2d kTargetPos4 = new Pose2d(8.259, 4.138, Rotation2d.fromDegrees(135));
        public static final Transform2d kCameraToRobot = new Transform2d(
            new Translation2d(-0.008486, -0.403435),
            new Rotation2d(Units.degreesToRadians(180))
        );
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

    public static class BlinkinPriorities{
        public static int kFlywheelReady = 5;
        public static int kFlywheelActive = 4;
        public static int kBallDetected = 3;
        public static int kIntakeActive = 2;
        public static int kIdle = 1;
    }
    public static class DriverConfigs{
        public static final double kTurnDeadband = 0.1;
        public static final double kDriveDeadband = 0.1;
    }
    
}
