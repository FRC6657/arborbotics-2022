// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera mLimelight = new PhotonCamera("limelight");
  private double yaw;
  private double pitch;
  private double distance;
  private boolean hasTargets;
  public final VisionSupplier visionSupplier = new VisionSupplier();
  private PhotonPipelineResult result;

  public static final double camDiagFOV = 67.8; // degrees
  public static final double camPitch = 42; // degrees
  public static final double camHeightOffGround = 0.637519; // meters
  public static final double maxLEDRange = 20; // meters
  public static final int camResolutionWidth = 320; // pixels
  public static final int camResolutionHeight = 240; // pixels
  public static final double minTargetArea = 10; // square pixels

  @Override
  public void periodic() {
    result = mLimelight.getLatestResult();
    if (result.hasTargets()) {
      hasTargets = true;
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      distance = PhotonUtils.calculateDistanceToTargetMeters(
          Constants.Vision.kCameraHeightMeters,
          Constants.Vision.kTargetHeightMeters,
          Constants.Vision.kCameraPitchRadians,
          Units.degreesToRadians(pitch));
    } else {
      hasTargets = false;
    }
  }

  // Vision supplier class
  public class VisionSupplier {

    public double getYaw() {
      return yaw;
    }

    // Method to get the pitch
    public double getPitch() {
      return pitch;
    }

    // Method to get the distance
    public double getDistance() {
      return distance;
    }

    // Method to check whether vision has targets
    public boolean hasTarget() {
      return hasTargets;
    }

    public PhotonPipelineResult getResult() {
      return result;
    }

    // Toggles the LL LEDs
    public void toggleLEDs() {
      if (RobotBase.isReal()) {
        if(mLimelight.getLEDMode() == VisionLEDMode.kOn){
          mLimelight.setLED(VisionLEDMode.kOff);
        }
        else{
          mLimelight.setLED(VisionLEDMode.kOn);
        }
      }
    }
  }
}
