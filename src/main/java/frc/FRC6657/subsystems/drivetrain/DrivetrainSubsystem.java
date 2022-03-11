// CopyRight (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.ArborMath;
import frc.FRC6657.custom.controls.Deadbander;
import frc.FRC6657.custom.controls.DriverProfile;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DrivetrainSubsystem extends SubsystemBase implements Loggable {

  // Drivetrain Motors
  private final WPI_TalonFX mFrontLeft, mFrontRight, mBackLeft, mBackRight;
  // Simulated Drivetrain Motors
  private TalonFXSimCollection mLeftSim, mRightSim;
  
  //Gyro
  @Log.Gyro(rowIndex = 2, columnIndex = 0, width = 2, height = 2, name = "Gyro", tabName = "Drivetrain")
  private final WPI_Pigeon2 mPigeon = new WPI_Pigeon2(Constants.kPigeonID);
  //Simulated Gyro
  private BasePigeonSimCollection mPigeonSim;

  //Kinematics and PoseEstimator Classes
  private final DifferentialDriveKinematics mKinematics;
  private final DifferentialDrivePoseEstimator mPoseEstimator = new DifferentialDrivePoseEstimator(
    new Rotation2d(), 
    new Pose2d(), 
    new MatBuilder<>( 
      Nat.N5(), 
      Nat.N1()
    ).fill(0.02, 0.02, 0.01, 0.02, 0.02), 
    new MatBuilder<>(
        Nat.N3(), 
        Nat.N1()
    ).fill(0.02, 0.02, 0.01), //Sensor Deviation Left, Right, Gyro
    new MatBuilder<>(
      Nat.N3(), 
      Nat.N1()
    ).fill(0.1, 0.1, 0.01) //Vision Measurement Deviation X, Y, Theta
  );

  //Feed forward and PID controller for advanced movement
  private final LinearPlantInversionFeedforward<N2,N2,N2> mFeedForward;
  private final PIDController mLinearPIDController;

  // Ramsete Controller for Path Following
  private RamseteController mRamseteController = new RamseteController();

  //Field2D for position visualization
  private Field2d mField = new Field2d();

  // Field objects to display trajectory following accuracy
  private FieldObject2d mTrajectoryPlot = mField.getObject("trajectory");
  private FieldObject2d mRobotPath = mField.getObject("robot-path");
  private List<Pose2d> mPathPoints = new ArrayList<Pose2d>();

  private FieldObject2d mIntakeVisualier = mField.getObject("intake");

  // Drivetrain Simulation
  DifferentialDrivetrainSim mDrivetrainSim;

  // Driver Profile
  private final DriverProfile mProfile;

  private VisionSupplier mVision;

  /*
   * 
   * Setup Methods
   *
   */

   /**
    * The subsystem that controls the drivetrain
    */
  public DrivetrainSubsystem(DriverProfile profile, VisionSupplier vision) {

    // Loads the Driver Profile
    this.mProfile = profile;
    this.mVision = vision;

    // Left Motor Assignments
    mFrontLeft = new WPI_TalonFX(Constants.kFrontLeftID);
    mBackLeft = new WPI_TalonFX(Constants.kBackLeftID);
    // Right Motor Assignments
    mFrontRight = new WPI_TalonFX(Constants.kFrontRightID);
    mBackRight = new WPI_TalonFX(Constants.kBackRightID);

    // Configures the motors
    configureMotors();

    //Gyro Stuff
    mPigeon.reset();

    // Fancy Stuff
    mKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.kTrackWidth);

    // Fancier Stuff
    mFeedForward = Constants.Drivetrain.kFeedForward;
    mLinearPIDController = Constants.Drivetrain.kLinearPIDController;

    // Simulation Stuff
    if (RobotBase.isSimulation()) {
      mDrivetrainSim = Constants.Drivetrain.kSim;
      mPigeonSim = mPigeon.getSimCollection();
      mLeftSim = mFrontLeft.getSimCollection();
      mRightSim = mFrontRight.getSimCollection();

    }

    // Field Visualization
    SmartDashboard.putData(mField);
  }

  /**
   * Configues the drivetrain motors
   */
  public void configureMotors() {

    // Reset motors to default
    mFrontLeft.configFactoryDefault();
    mFrontRight.configFactoryDefault();
    mBackLeft.configFactoryDefault();
    mBackRight.configFactoryDefault();

    // Set the back motors to follow the commands of the front
    mBackLeft.follow(mFrontLeft);
    mBackRight.follow(mFrontRight);

    // Set the neutral modes
    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);

    // Makes Green go Forward. Sim is weird so thats what the if statement is for
    if (RobotBase.isReal()) {
      mFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);
      mBackLeft.setInverted(TalonFXInvertType.FollowMaster);
      mFrontRight.setInverted(TalonFXInvertType.Clockwise);
      mBackRight.setInverted(TalonFXInvertType.FollowMaster);
    } else {
      mFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);
      mBackLeft.setInverted(TalonFXInvertType.FollowMaster);
      mFrontRight.setInverted(TalonFXInvertType.CounterClockwise);
      mBackRight.setInverted(TalonFXInvertType.FollowMaster);
    }

    // Encoders
    mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Reduce the encoder delay
    mFrontLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    mFrontRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    mFrontLeft.configVelocityMeasurementWindow(1);
    mFrontRight.configVelocityMeasurementWindow(1);

    // Limits the current to prevent breaker tripping
    mFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5)); // | Enabled | 60a
                                                                                                 // Limit | 65a Thresh |
                                                                                                 // .5 sec Trigger Time
    mFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5));// | Enabled | 60a
                                                                                                 // Limit | 65a Thresh |
                                                                                                 // .5 sec Trigger Time
    mBackLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5)); // | Enabled | 60a Limit
                                                                                                // | 65a Thresh | .5 sec
                                                                                                // Trigger Time
    mBackRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5)); // | Enabled | 60a
                                                                                                 // Limit | 65a Thresh |
                                                                                                 // .5 sec Trigger Time

    // Turn off unused things to reduce CAN Utilization
    mBackLeft.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    mBackRight.setStatusFramePeriod(StatusFrame.Status_1_General, 250);

    mBackLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
    mBackRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    mFrontLeft.setStatusFramePeriod(StatusFrame.Status_10_Targets, 250);
    mFrontRight.setStatusFramePeriod(StatusFrame.Status_10_Targets, 250);
    mBackLeft.setStatusFramePeriod(StatusFrame.Status_10_Targets, 250);
    mBackRight.setStatusFramePeriod(StatusFrame.Status_10_Targets, 250);

    mBackLeft.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 250);
    mBackRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 250);

    mFrontLeft.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 250);
    mFrontRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 250);
    mBackLeft.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 250);
    mBackRight.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 250);

    mFrontLeft.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 250);
    mFrontRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 250);
    mBackLeft.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 250);
    mBackRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 250);

    mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 250);
    mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 250);

  }

  /*
   * 
   * Reset Methods
   * 
   */
  /**
   * Resets the odometry
   * 
   * @param pose New Position
   */
  public void resetOdometry(Pose2d pose) {
    // Reset Simulation entirely if we are in simulation
    if (RobotBase.isSimulation()) {
      mDrivetrainSim = Constants.Drivetrain.kSim;
    }

    resetEncoders();// Reset the encoders


  }
  /**
   * Resets the encoders
   */
  public void resetEncoders() {
    mFrontLeft.setSelectedSensorPosition(0);
    mFrontRight.setSelectedSensorPosition(0);
  }

  /**
   * Resets the gyro
   */
  public void resetGyro(){
    mPigeon.reset();
    mPoseEstimator.resetPosition(mPoseEstimator.getEstimatedPosition(), mPigeon.getRotation2d());
  }
  
  public void resetPoseEstimator(Pose2d newPose){
    mPoseEstimator.resetPosition(newPose, mPigeon.getRotation2d());
  }

  /**
   * @param xSpeed      Forward Input
   * @param zRotation   Turning Input
   * @param isQuickTurn Allow Quickturn?
   * @param modSpeed    Modify Speed?
   */
  public void teleopCurvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn, boolean modSpeed) {
    setCurvatureSpeeds(xSpeed, zRotation, isQuickTurn, modSpeed);
  }

  /*
   * 
   * Set Methods
   * 
   */

  /**
   * Sets motor voltage based on left and right velocities.
   * 
   * @param speeds Left and Right Velocities.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {

    var feedForward = mFeedForward.calculate(
      new MatBuilder<>(
        Nat.N2(), 
        Nat.N1()
      )
      .fill(
        speeds.leftMetersPerSecond, 
        speeds.rightMetersPerSecond
      )
    );

    final double leftOutput = mLinearPIDController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput = mLinearPIDController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);

    mFrontLeft.setVoltage(leftOutput + feedForward.get(0, 0));
    mFrontRight.setVoltage(rightOutput + feedForward.get(1, 0));
  }

  /**
   * Sets motor voltage based on left and right signals which get scaled to a max
   * speed
   * 
   * @param speeds Left and Right speeds input -1 to 1
   */
  public void setSpeeds(WheelSpeeds speeds) {
    setSpeeds(
      new DifferentialDriveWheelSpeeds(
        speeds.left * Constants.Drivetrain.kMaxAttainableSpeed,
        speeds.right * Constants.Drivetrain.kMaxAttainableSpeed
      )
    );
  }

  /**
   * @param xSpeed    Forward Signal
   * @param zRotation Turn Signal
   * @param quickturn Allow Quickturn?
   * @param modSpeed  Modify Speed?
   * 
   *                  Sets drivetrain speeds in m/s from inputed controls that
   *                  follow the curvature drive structure
   * 
   */
  public void setCurvatureSpeeds(double xSpeed, double zRotation, boolean quickturn, boolean modSpeed) {

    xSpeed = Deadbander.applyLinearScaledDeadband(xSpeed, Constants.DriverConfigs.kDriveDeadband);
    zRotation = Deadbander.applyLinearScaledDeadband(zRotation, Constants.DriverConfigs.kTurnDeadband);

    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds();
    WheelSpeeds wheelSpeeds = DifferentialDrive.curvatureDriveIK(xSpeed, zRotation, quickturn);

    if (modSpeed) {
      speeds.leftMetersPerSecond = wheelSpeeds.left * mProfile.kModSpeed;
      speeds.rightMetersPerSecond = wheelSpeeds.right * mProfile.kModSpeed;
    } else {
      speeds.leftMetersPerSecond = wheelSpeeds.left * mProfile.kMaxSpeed;
      speeds.rightMetersPerSecond = wheelSpeeds.right * mProfile.kMaxSpeed;
    }
    setSpeeds(speeds);
  }

  /**
   * Stops the Drivetrain
   */
  public void stop() {
    mFrontLeft.set(0);
    mFrontRight.set(0);
  }
  /*
   * Get Methods
   *
   * /

  /**
   * Get left drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "Left Distance", tabName = "DrivetrainSubsystem")
  public double getLeftMeters() {
    return mFrontLeft.getSelectedSensorPosition() * Constants.Drivetrain.kDistancePerPulse;
  }

  /**
   * Get right drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 2, width = 2, height = 1, name = "Right Distance", tabName = "DrivetrainSubsystem")
  public double getRightMeters() {
    return mFrontRight.getSelectedSensorPosition() * Constants.Drivetrain.kDistancePerPulse;
  }

  /**
   * Get left drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 0, width = 2, height = 1, name = "Left Velocity", tabName = "DrivetrainSubsystem")
  public double getLeftVelocity() {
    return mFrontLeft.getSelectedSensorVelocity() * 10 * Constants.Drivetrain.kDistancePerPulse;
  }

  /**
   * Get right drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 2, width = 2, height = 1, name = "Right Velocity", tabName = "DrivetrainSubsystem")
  public double getRightVelocity() {
    return mFrontRight.getSelectedSensorVelocity() * 10 * Constants.Drivetrain.kDistancePerPulse;
  }

  /**
   * This is mainly to have a velocity gauge on shuffleboard.
   * 
   * @return Same as getLeftVelocity()
   */
  @Log.Dial(rowIndex = 2, columnIndex = 2, width = 1, height = 1, name = "Left Vel", min = -Constants.Drivetrain.kMaxAttainableSpeed, max = Constants.Drivetrain.kMaxAttainableSpeed, showValue = false, tabName = "Drivetrain")
  public double leftVelocityGauge() {
    return getLeftVelocity();
  }

  /**
   * This is mainly to have a velocity gauge on shuffleboard.
   * 
   * @return Same as getRightVelocity()
   */
  @Log.Dial(rowIndex = 2, columnIndex = 3, width = 1, height = 1, name = "Right Vel", min = -Constants.Drivetrain.kMaxAttainableSpeed, max = Constants.Drivetrain.kMaxAttainableSpeed, showValue = false, tabName = "DrivetrainSubsystem")
  public double rightVelocityGauge() {
    return getRightVelocity();
  }

  @Log(rowIndex = 3, columnIndex = 0, width = 2, height = 1, name = "Gyro Velocity", tabName = "Drivetrain")
  public double getHeadingVelocity(){
    double[] gyroVals = {0,0,0};
    mPigeon.getRawGyro(gyroVals);
    return gyroVals[2];
  }
  /**
   * @return Gyro accumulated angle if it is ready to give accurate data
   */
  public double getGyroAngle(){
      return mPigeon.getYaw();
  }

  @Log(rowIndex = 0, columnIndex = 3, width = 2, height = 1, name = "Gyro Normalized", tabName = "Scott Gyro Stuff")
  public double getGyroCorrected() {
    return ArborMath.normalizeFusedHeading(mPigeon.getYaw());
  }

  public void updatePoseEstimator(DifferentialDriveWheelSpeeds actWheelSpeeds, double leftDist, double rightDist) {
    mPoseEstimator.update(mPigeon.getRotation2d(), actWheelSpeeds, leftDist, rightDist);
    var res = mVision.getResult();
    if(res.hasTargets()){
      double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
      Transform2d camToTargetTrans = res.getBestTarget().getCameraToTarget();
      Pose2d camPose = Constants.Vision.kTargetPos1.transformBy(camToTargetTrans.inverse());
      mPoseEstimator.addVisionMeasurement(camPose.transformBy(Constants.Vision.kCameraToRobot), imageCaptureTime);
    }
  }

  /*
   * Commands
   */

  public class TurnByAngleCommand extends CommandBase {

    double mSetpoint;
    double mStartPoint;

    final PIDController mPIDController = new PIDController(Constants.Drivetrain.Turn_Command_kP, 0, Constants.Drivetrain.Turn_Command_kD);

   /**
    * Turn a specified Degree Amount
    */
    public TurnByAngleCommand(double mSetpoint) {
      this.mSetpoint = mSetpoint;
    }

    @Override
    public void initialize() {
      this.mStartPoint = getGyroAngle();
    }

    @Override
    public void execute() {
      double output = mPIDController.calculate(getGyroAngle(), mStartPoint + mSetpoint);
      mFrontLeft.set(-output);
      mFrontRight.set(output);

    }

    @Override
    public boolean isFinished() {
      return Math.abs((mStartPoint + mSetpoint) - getGyroAngle()) <= Constants.Drivetrain.kTurnCommandTolerance;
    }

  }
  /**
   * Command to follow a given trajectory
   */
  public class TrajectoryFollowerCommand extends CommandBase {

    private final Timer timer = new Timer();
    private final Trajectory trajectory;

    public TrajectoryFollowerCommand(Trajectory trajectory) {
      this.trajectory = trajectory;
      addRequirements(DrivetrainSubsystem.this);
    }

    @Override
    public void initialize() {
      mPathPoints.clear();
      mPathPoints.add(mPoseEstimator.getEstimatedPosition());
      mRobotPath.setPoses(mPathPoints);
      mTrajectoryPlot.setTrajectory(trajectory);

      timer.reset();
      timer.start();
    }

    @Override
    public void execute() {

        mPathPoints.add(mPoseEstimator.getEstimatedPosition());
        mRobotPath.setPoses(mPathPoints);

        State desiredPose = trajectory.sample(timer.get());
        ChassisSpeeds refChassisSpeeds = mRamseteController.calculate(mPoseEstimator.getEstimatedPosition(), desiredPose);
        DifferentialDriveWheelSpeeds wheelSpeeds = mKinematics.toWheelSpeeds(refChassisSpeeds);

      setSpeeds(wheelSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
      stop();
    }

    @Override
    public boolean isFinished() {
      return timer.get() > trajectory.getTotalTimeSeconds();
    }
  }

  public class VisionAimAssist extends CommandBase{
    PIDController mPID = new PIDController(Constants.Drivetrain.vision_kP, 0, Constants.Drivetrain.vision_kD);

    @Override
    public void initialize() {
        mVision.enableLEDs();
    }

    @Override
    public void execute() {
      if(mVision.hasTarget()){
        double effort = mPID.calculate(mVision.getYaw(), 0);
        mFrontLeft.setVoltage(-effort);
        mFrontRight.setVoltage(effort);
      }
    }
    @Override
    public void end(boolean interrupted) {
        stop();
        mVision.disableLEDs();
    }
  }


  /*
   *
   * WPILib Methods
   * 
   */

  @Override
  public void periodic() {
    updatePoseEstimator(new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()), getLeftMeters(), getRightMeters());
    mField.setRobotPose(mPoseEstimator.getEstimatedPosition());

    if(RobotBase.isSimulation()){
      mIntakeVisualier.setPose(
        new Pose2d(
          (mPoseEstimator.getEstimatedPosition().getX() + 0.6056505 * Math.cos(Units.degreesToRadians(mPoseEstimator.getEstimatedPosition().getRotation().getDegrees()))),
          (mPoseEstimator.getEstimatedPosition().getY() + 0.6056505 * Math.sin(Units.degreesToRadians(mPoseEstimator.getEstimatedPosition().getRotation().getDegrees()))),
          mPoseEstimator.getEstimatedPosition().getRotation()
        )
      );
    }
  }

  @Override
  public void simulationPeriodic() {

    mDrivetrainSim.setInputs(mFrontLeft.get() * RobotController.getInputVoltage(),
        mFrontRight.get() * RobotController.getInputVoltage());

    mDrivetrainSim.update(0.02);

    mLeftSim.setIntegratedSensorRawPosition(
        (int) (mDrivetrainSim.getLeftPositionMeters() / Constants.Drivetrain.kDistancePerPulse));
    mRightSim.setIntegratedSensorRawPosition(
        (int) (mDrivetrainSim.getRightPositionMeters() / Constants.Drivetrain.kDistancePerPulse));
    mLeftSim.setIntegratedSensorVelocity(
        (int) (mDrivetrainSim.getLeftVelocityMetersPerSecond() / (10 * Constants.Drivetrain.kDistancePerPulse)));
    mRightSim.setIntegratedSensorVelocity(
        (int) (mDrivetrainSim.getRightVelocityMetersPerSecond() / (10 * Constants.Drivetrain.kDistancePerPulse)));

    mPigeonSim.setRawHeading(mDrivetrainSim.getHeading().getDegrees());

    mVision.processSim(mPoseEstimator.getEstimatedPosition());

  }
}
