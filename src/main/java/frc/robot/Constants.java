package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;

public final class Constants {

    public static final boolean logging = true;

    public static class DriveProfile{
        public static final double kMaxDriveSpeed = 10; //Meters/s
        public static final double kMaxTurnSpeed = Units.degreesToRadians(360*3); //Rad/s

        public static final double kModDriveSpeed = 5; //Meters/s

        public static final double kDriveForwardAccel = 5; //Meters/s^2
        public static final double kDriveBackwardAccel = 5; //Meters/s^2
        public static final double kDriveForwardDecel = 5; //Meters/s^2
        public static final double kDriveBackwardDecel = 5; //Meters/s^2
        public static final double kTurnAccel = Units.degreesToRadians(360*3); //Rad/s^2
    }

    public static class CAN{
        public static final int kDrive_FrontLeft = 1;
        public static final int kDrive_FrontRight = 2;
        public static final int kDrive_BackLeft = 3;
        public static final int kDrive_BackRight = 4;
        public static final int kPigeon = 5;
        public static final int kIntake = 6;
        public static final int kPCM = 7;
        public static final int kFlywheelMaster = 8;
        public static final int kFlywheelSlave = 9;
        public static final int kAccelerator = 10;
        public static final int kHood = 11;
        public static final int kRightLift = 12; 
        public static final int kLeftLift = 13;
    }

    public static class Accelerator {
        public static final double kSpeed = 1.0;
    }

    public static class Intake {
        public static final double kSpeed = .75;
    }

    public static class Flywheel {
        public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(0.8, 12d/(6380d*1.5d));
        public static final double kRPMConversionFactor = (600d/2048d) * 1.5;
        public static final double kRPMTolerance = 50;
    }   

    public static class Hood {
        public static final double kAngleConversionFactor = ((((10*5*3)*(2/3d)*(5/2d))))/360d;
    }

    public static class Drivetrain {
        
        public static final double kTrackwidth = 0.61568;
        public static final double kDistancePerPulse = (2 * Math.PI * Units.inchesToMeters(3)) / (2048 * KitbotGearing.k10p71.value); // Conversion between Counts and Meters

    }
    
    public static class Vision{
        public static final String kCameraName = "limelight";
        public static final double kCameraHeightMeters = 0.638374;// CAD Estimate
        public static final double kTargetHeightMeters = Units.feetToMeters(8 + 8 / 12); // Field Vision Target 
        //public static final double kTargetHeightMeters = Units.inchesToMeters(81.75); // Y4 Vision Target
        public static final double kCameraPitchRadians = Units.degreesToRadians(42);
        public static final double kMaxLEDRange = 200;
        public static final double kCamDiagFOV = 67.8; // degrees
        public static final int kCamResolutionWidth = 320; // pixels
        public static final int kCamResolutionHeight = 240; // pixels
        public static final double kTargetWidth = Units.feetToMeters(4);  
        public static final double kMinTargetArea = 0;
        public static final Pose2d kTargetPos = new Pose2d(8.259, 4.138, Rotation2d.fromDegrees(0));
        public static final Transform2d kCameraToRobot = new Transform2d(
            new Translation2d(-0.008486, -0.403435),
            new Rotation2d(Units.degreesToRadians(180))
        );
    }
}
