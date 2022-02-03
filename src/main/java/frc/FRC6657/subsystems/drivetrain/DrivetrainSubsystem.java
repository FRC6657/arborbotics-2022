// CopyRight (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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
import frc.FRC6657.Constants.Triangle;
import frc.FRC6657.custom.ctre.SendablePigeonIMU;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DrivetrainSubsystem extends SubsystemBase implements Loggable {

  private final WPI_TalonFX mFrontLeft, mFrontRight, mBackLeft, mBackRight;
  private TalonFXSimCollection mLeftSim, mRightSim;

  @Log.Gyro(rowIndex = 2, columnIndex = 0, width = 2, height = 2, name = "Gyro")
  private final SendablePigeonIMU mPigeonIMU;
  private BasePigeonSimCollection mPigeonIMUSim;

  private final DifferentialDriveKinematics mKinematics;
  private final DifferentialDriveOdometry mOdometry;

  private final SimpleMotorFeedforward mFeedForward;
  private final PIDController mLinearPIDController;
  private final PIDController mAngularPIDController;

  private RamseteController mRamseteController = new RamseteController();

  private Field2d mField = new Field2d();

  private FieldObject2d mTrajectoryPlot = mField.getObject("trajectory");
  private FieldObject2d mRobotPath = mField.getObject("robot-path");
  private List<Pose2d> mPathPoints = new ArrayList<Pose2d>();

  private FieldObject2d mTarget = mField.getObject("target");
  private FieldObject2d mTriangle = mField.getObject("triangle");
  private List<Pose2d> mTrianglePoints = new ArrayList<Pose2d>();

  private FieldObject2d mAngle = mField.getObject("angle");
  private List<Pose2d> mAnglePoints = new ArrayList<Pose2d>();

  DifferentialDrivetrainSim mDrivetrainSim;
  /*
   * 
   * Setup Methods
   *
   */

  /**
   * The subsystem that controls the drivetrain
   */
  public DrivetrainSubsystem() {

    // Left Stuff
    mFrontLeft = new WPI_TalonFX(Constants.kFrontLeftID);
    mBackLeft = new WPI_TalonFX(Constants.kBackLeftID);
    // Right Stuff
    mFrontRight = new WPI_TalonFX(Constants.kFrontRightID);
    mBackRight = new WPI_TalonFX(Constants.kBackRightID);

    // Configures the motors
    configureMotors();

    // Gyro Stuff
    mPigeonIMU = new SendablePigeonIMU(Constants.kPigeonID);
    mPigeonIMU.reset();

    // Fancy Stuff
    mKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.kTrackWidth);
    mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroAngle()));

    // Fancier Stuff
    mFeedForward = Constants.Drivetrain.kFeedForward;
    mLinearPIDController = Constants.Drivetrain.kLinearPIDController;
    mAngularPIDController = Constants.Drivetrain.kAngularPIDController;

    // Simulation Stuff
    if (RobotBase.isSimulation()) {
      mDrivetrainSim = Constants.Drivetrain.kSim;
      mPigeonIMUSim = mPigeonIMU.getSimCollection();
      mLeftSim = mFrontLeft.getSimCollection();
      mRightSim = mFrontRight.getSimCollection();
    }

    mTarget.setPose(new Pose2d(Constants.Drivetrain.kTargetX,Constants.Drivetrain.kTargetY,new Rotation2d()));
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
    mBackLeft.setNeutralMode(NeutralMode.Coast);
    mBackRight.setNeutralMode(NeutralMode.Coast);

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

    // More precise encoders
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

    if (RobotBase.isSimulation()) {
      mDrivetrainSim = Constants.Drivetrain.kSim;
    }

    resetEncoders();
    mOdometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));

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
  public void resetGyro() {
    mPigeonIMU.reset();
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
    final double leftFeedforward = mFeedForward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = mFeedForward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = mLinearPIDController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput = mLinearPIDController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);

    mFrontLeft.setVoltage(leftOutput + leftFeedforward);
    mFrontRight.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Sets motor voltage based on left and right signals which get scaled to a max
   * speed
   * 
   * @param speeds Left and Right speeds input -1 to 1
   */
  public void setSpeeds(WheelSpeeds speeds) {

    speeds.left *= Constants.Drivetrain.kMaxSpeed;
    speeds.right *= Constants.Drivetrain.kMaxSpeed;

    final double leftFeedforward = mFeedForward.calculate(speeds.left);
    final double rightFeedforward = mFeedForward.calculate(speeds.right);

    final double leftOutput = mLinearPIDController.calculate(getLeftVelocity(), speeds.left);
    final double rightOutput = mLinearPIDController.calculate(getRightVelocity(), speeds.right);

    mFrontLeft.setVoltage(leftOutput + leftFeedforward);
    mFrontRight.setVoltage(rightOutput + rightFeedforward);
  }

  public void setTurnSpeed(WheelSpeeds speeds) {

    speeds.left *= Constants.Drivetrain.kTurnSpeed*(Math.PI*Constants.Drivetrain.kTrackWidth);
    speeds.right *= Constants.Drivetrain.kTurnSpeed*(Math.PI*Constants.Drivetrain.kTrackWidth);

    final double leftFeedforward = mFeedForward.calculate(speeds.left);
    final double rightFeedforward = mFeedForward.calculate(speeds.right);

    final double leftOutput = mLinearPIDController.calculate(getLeftVelocity(), speeds.left);
    final double rightOutput = mLinearPIDController.calculate(getRightVelocity(), speeds.right);

    mFrontLeft.setVoltage(leftOutput + leftFeedforward);
    mFrontRight.setVoltage(rightOutput + rightFeedforward);
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
   */

  /**
   * @return Robot Position
   */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * Get left drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "Left Distance")
  public double getLeftMeters() {
    return mFrontLeft.getSelectedSensorPosition() * Constants.Drivetrain.kDistancePerPulse;
  }

  /**
   * Get right drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 2, width = 2, height = 1, name = "Right Distance")
  public double getRightMeters() {
    return mFrontRight.getSelectedSensorPosition() * Constants.Drivetrain.kDistancePerPulse;
  }

  /**
   * Get left drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 0, width = 2, height = 1, name = "Left Velocity")
  public double getLeftVelocity() {
    return mFrontLeft.getSelectedSensorVelocity() * 10 * Constants.Drivetrain.kDistancePerPulse;
  }

  /**
   * Get right drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 2, width = 2, height = 1, name = "Right Velocity")
  public double getRightVelocity() {
    return mFrontRight.getSelectedSensorVelocity() * 10 * Constants.Drivetrain.kDistancePerPulse;
  }

  /**
   * Get temperature of the front left motor in celsius
   */
  @Log.Dial(rowIndex = 0, columnIndex = 4, width = 2, height = 2, name = "FL Temp", max = 110, min = 20, showValue = false)
  public double getFrontLeftTemp() {
    if (RobotBase.isReal()) {
      return mFrontLeft.getTemperature();
    }
    return 0;
  }

  /**
   * Get temperature of the front right motor in celsius
   */
  @Log.Dial(rowIndex = 0, columnIndex = 6, width = 2, height = 2, name = "FR Temp", max = 110, min = 20, showValue = false)
  public double getFrontRightTemp() {
    if (RobotBase.isReal()) {
      return mFrontRight.getTemperature();
    }
    return 0;
  }

  /**
   * Get temperature of the back left motor in celsius
   */
  @Log.Dial(rowIndex = 2, columnIndex = 4, width = 2, height = 2, name = "BL Temp", max = 110, min = 20, showValue = false)
  public double getBackLeftTemp() {
    if (RobotBase.isReal()) {
      return mBackLeft.getTemperature();
    }
    return 0;
  }

  /**
   * Get temperature of the back right motor in celsius
   */
  @Log.Dial(rowIndex = 2, columnIndex = 6, width = 2, height = 2, name = "BR Temp", max = 110, min = 20, showValue = false)
  public double getBackRightTemp() {
    if (RobotBase.isReal()) {
      return mBackRight.getTemperature();
    }
    return 0;
  }

  /**
   * This is mainly to have a velocity gauge on shuffleboard.
   * 
   * @return Same as getLeftVelocity()
   */
  @Log.Dial(rowIndex = 2, columnIndex = 2, width = 1, height = 1, name = "Left Vel", min = -Constants.Drivetrain.kMaxSpeed, max = Constants.Drivetrain.kMaxSpeed, showValue = false)
  public double leftVelocityGauge() {
    return getLeftVelocity();
  }

  /**
   * This is mainly to have a velocity gauge on shuffleboard.
   * 
   * @return Same as getRightVelocity()
   */
  @Log.Dial(rowIndex = 2, columnIndex = 3, width = 1, height = 1, name = "Right Vel", min = -Constants.Drivetrain.kMaxSpeed, max = Constants.Drivetrain.kMaxSpeed, showValue = false)
  public double rightVelocityGauge() {
    return getRightVelocity();
  }

  public double getGyroAngle() {
    if (mPigeonIMU.getState() == PigeonState.Ready) {
      return mPigeonIMU.getFusedHeading();
    }
    return 0;
  }

  public double getEstTargetAngleError() {
    Triangle curTriangle;
    double xOffset; // Meters
    double yOffset; // Meters
    xOffset = Constants.Drivetrain.kTargetX - mOdometry.getPoseMeters().getX();
    yOffset = Constants.Drivetrain.kTargetY - mOdometry.getPoseMeters().getY();
    curTriangle = new Triangle(xOffset, yOffset);

    SmartDashboard.putNumber("xOffset", xOffset);
    SmartDashboard.putNumber("yOffset", yOffset);
    SmartDashboard.putNumber("Tri-Angle", curTriangle.getAngle());

    return 0.0;
  }

  public double getOdemetryDegrees(){
    if(mOdometry.getPoseMeters().getRotation().getDegrees() < 0){
      return 180 + (180 + mOdometry.getPoseMeters().getRotation().getDegrees());
    }
    else {
      return mOdometry.getPoseMeters().getRotation().getDegrees();
    }
  }

  /*
   * Commands
   */

  /**
   * Command for TeleOp Driving
   */
  public class DriveCommand extends CommandBase {

    private DoubleSupplier xSpeed;
    private DoubleSupplier zRotation;
    private BooleanSupplier isQuickturn;

    public DriveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier isQuickturn) {
      this.xSpeed = xSpeed;
      this.zRotation = zRotation;
      this.isQuickturn = isQuickturn;
      addRequirements(DrivetrainSubsystem.this);
    }

    @Override
    public void initialize() {
      System.out.println("Driver Control Initialized");
    }

    @Override
    public void execute() {
      setSpeeds(DifferentialDrive.curvatureDriveIK(xSpeed.getAsDouble(), zRotation.getAsDouble(),
          isQuickturn.getAsBoolean()));
    }

    @Override
    public void end(boolean interrupted) {
      stop();
    }
  }

  /**
   * Command to follow a given trajectory
   */
  public class TrajectoryFollowerCommand extends CommandBase {

    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private boolean resetPose;

    public TrajectoryFollowerCommand(Trajectory trajectory, boolean resetPose) {
      this.trajectory = trajectory;
      this.resetPose = resetPose;
      addRequirements(DrivetrainSubsystem.this);
    }

    @Override
    public void initialize() {

      if (resetPose) {
        resetOdometry(trajectory.getInitialPose());
        mField.setRobotPose(trajectory.getInitialPose());
      }

      mPathPoints.clear();
      mPathPoints.add(mOdometry.getPoseMeters());
      mRobotPath.setPoses(mPathPoints);
      mTrajectoryPlot.setTrajectory(trajectory);

      timer.start();
    }

    @Override
    public void execute() {
      mPathPoints.add(mOdometry.getPoseMeters());
      mRobotPath.setPoses(mPathPoints);

      State desiredPose = trajectory.sample(timer.get());
      ChassisSpeeds refChassisSpeeds = mRamseteController.calculate(mOdometry.getPoseMeters(), desiredPose);
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

  public class VisionAimCommand extends CommandBase {

    public final double turnError;
    public final double distanceError;
    public final boolean hasTarget;

    public VisionAimCommand(double turnError, double distanceError, boolean hasTarget) {
      this.turnError = turnError;
      this.distanceError = distanceError;
      this.hasTarget = hasTarget;
    }

    @Override
    public void execute() {
      if (hasTarget) {
        WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(distanceError,
            -mAngularPIDController.calculate(turnError, 0), false);
        mFrontLeft.set(speeds.left);
        mFrontRight.set(speeds.right);
      }
    }

    @Override
    public void end(boolean interrupted) {
      stop();
    }

    @Override
    public boolean isFinished() {
      return (turnError < Constants.Drivetrain.kAimTollerance
          && distanceError < Constants.Drivetrain.kDistanceTollerance) || !hasTarget;
    }

  }

  public class OdometryAimCommand extends CommandBase {

    public OdometryAimCommand(){
      addRequirements(DrivetrainSubsystem.this);
    }
    
    @Override
    public void execute() {
      double error = getEstTargetAngleError()/180;
      System.out.println(error);
      setTurnSpeed(new WheelSpeeds(error,-error));
    }


    @Override
    public void end(boolean interrupted) {
        stop();
    }

    @Override
    public boolean isFinished() {
      //return(Math.abs(getEstTargetAngleError())<5);
      return false;
    }

  }

  /*
   *
   * WPILib Methods
   * 
   */

  @Override
  public void periodic() {
    mOdometry.update(Rotation2d.fromDegrees(getGyroAngle()), getLeftMeters(), getRightMeters());
    mField.setRobotPose(mOdometry.getPoseMeters());

    mTrianglePoints.clear();

    mTrianglePoints.add(mOdometry.getPoseMeters());
    mTrianglePoints.add(new Pose2d(Constants.Drivetrain.kTargetX, mOdometry.getPoseMeters().getY(), new Rotation2d()));
    mTrianglePoints.add(new Pose2d(Constants.Drivetrain.kTargetX,Constants.Drivetrain.kTargetY, new Rotation2d()));

    mTriangle.setPoses(mTrianglePoints);


    double angleRadius = 4;

    mAnglePoints.clear();
    mAnglePoints.add(mOdometry.getPoseMeters());
    mAnglePoints.add(
      new Pose2d(
        (mOdometry.getPoseMeters().getX() + angleRadius * Math.cos(Units.degreesToRadians(getOdemetryDegrees()))),
        (mOdometry.getPoseMeters().getY() + angleRadius * Math.sin(Units.degreesToRadians(getOdemetryDegrees()))),
        new Rotation2d()
      )
    );

    mAngle.setPoses(mAnglePoints);

    SmartDashboard.putNumber("Angle", getOdemetryDegrees());

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

    mPigeonIMUSim.setRawHeading(mDrivetrainSim.getHeading().getDegrees());
  }
}
