// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera mLimelight = new PhotonCamera(Constants.Vision.kCameraName);
  private double yaw;
  private double pitch;
  private double distance;
  private boolean hasTargets;
  public final VisionSupplier visionSupplier = new VisionSupplier();
  private PhotonPipelineResult result;

  public static final Translation2d kFieldCenterX = new Translation2d(8.2295, 4.115);
  
  private SimVisionSystem mSim = new SimVisionSystem(
    Constants.Vision.kCameraName,
    Constants.Vision.kCamDiagFOV,
    Units.radiansToDegrees(Constants.Vision.kCameraPitchRadians),
    Constants.Vision.kCameraToRobot,
    Constants.Vision.kCameraHeightMeters,
    Constants.Vision.kMaxLEDRange,
    Constants.Vision.kCamResolutionWidth,
    Constants.Vision.kCamResolutionHeight,
    Constants.Vision.kMinTargetArea
  );

  private SimVisionTarget mTarget = new SimVisionTarget(Constants.Vision.kTargetPos, Constants.Vision.kTargetHeightMeters, Units.feetToMeters(3), Units.inchesToMeters(2.5));

  public VisionSubsystem(){
    mSim.addSimVisionTarget(mTarget);
  }


  @Override
  public void periodic() {

    SmartDashboard.putNumber("Yaw", yaw);

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

    public SimVisionSystem getSim(){
      return mSim;
    }

    public void processSim(Pose2d robotPoseMeters){
      mSim.processFrame(robotPoseMeters);
    }

    public void setDriverMode(boolean enabled){
      mLimelight.setDriverMode(enabled);
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

    public void enableLEDs(){
      mLimelight.setLED(VisionLEDMode.kOn);
    }
    public void disableLEDs(){
      mLimelight.setLED(VisionLEDMode.kOff);
    }

  }
}
