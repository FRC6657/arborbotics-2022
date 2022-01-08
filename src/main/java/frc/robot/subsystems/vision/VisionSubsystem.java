// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Vision subsystem
 * This subsystem contains and updates vision information gathered from Photon
 * vision
 */

public class VisionSubsystem extends SubsystemBase {

  // Variables
  public final PhotonCamera camera = new PhotonCamera("limelight");
  private double yaw;
  private double pitch;
  private double distance;
  private boolean hasTargets;
  public final VisionSupplier visionSupplier = new VisionSupplier();

  // Periodic method
  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      hasTargets = true;
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      distance = PhotonUtils.calculateDistanceToTargetMeters(
          Constants.cameraHeightMeters,
          Constants.targetHeightMeters,
          Constants.cameraPitchRadians,
          Units.degreesToRadians(pitch));
    } else {
      hasTargets = false;
    }
  }

  // Vision supplier class
  public class VisionSupplier {

    public double getYaw() {
      return -yaw;
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

  }

}