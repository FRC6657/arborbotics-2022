// CopyRight (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.custom.ctre.LazyTalonFX;
import frc.robot.custom.ctre.SendablePigeonIMU;
import frc.robot.custom.ctre.TalonEncoder;
import frc.robot.custom.ctre.TalonEncoderSim;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DrivetrainSubsystem extends SubsystemBase implements Loggable{

  private final LazyTalonFX mFrontLeft, mFrontRight, mBackLeft, mBackRight; 
  private final TalonEncoder mLeftEncoder, mRightEncoder;
  private TalonEncoderSim mLeftSimcoder, mRightSimcoder;
  
  @Log.Gyro(rowIndex = 2, columnIndex = 0, width = 2, height = 2, name = "Gyro")
  private final SendablePigeonIMU mPigeonIMU;
  private BasePigeonSimCollection mPigeonIMUSim;

  private final DifferentialDriveKinematics mKinematics;
  private final DifferentialDrivePoseEstimator mDifferentialDrivePoseEstimator;

  private final SimpleMotorFeedforward mFeedForward;
  private final PIDController mPIDController;

  private RamseteController mRamseteController = new RamseteController();

  private Field2d mField = new Field2d();
  
  private FieldObject2d mTrajectoryPlot = mField.getObject("trajectory");
  private FieldObject2d mRobotPath = mField.getObject("robot-path");
  private List<Pose2d> mPathPoints = new ArrayList<Pose2d>();

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

    //Left Stuff
    mFrontLeft = new LazyTalonFX(Constants.kFrontLeftID);
    mBackLeft = new LazyTalonFX(Constants.kBackLeftID);
    mLeftEncoder = new TalonEncoder(mFrontLeft);
    mLeftEncoder.setDistancePerPulse(Constants.kDistancePerPulse);
    mLeftEncoder.reset();

    //Right Stuff
    mFrontRight = new LazyTalonFX(Constants.kFrontRightID);
    mBackRight = new LazyTalonFX(Constants.kBackRightID);
    mRightEncoder = new TalonEncoder(mFrontRight);
    mRightEncoder.setDistancePerPulse(Constants.kDistancePerPulse);
    mRightEncoder.reset();

    //Configures the motors
    configureMotors();

    //Gyro Stuff
    mPigeonIMU = new SendablePigeonIMU(Constants.kPigeonID);
    mPigeonIMU.reset();

    //Fancy Stuff
    mKinematics = new DifferentialDriveKinematics(Constants.kTrackWidth);
    mDifferentialDrivePoseEstimator = new DifferentialDrivePoseEstimator( //TODO Tune this 
      mPigeonIMU.getRotation2d(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01),
      VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    //Fancier Stuff
    mFeedForward = Constants.kFeedForward;
    mPIDController = Constants.kDrivePIDController;

    //Field Visualization
    SmartDashboard.putData(mField);

    //Simulation Stuff
    if(RobotBase.isSimulation()){
      mDrivetrainSim = Constants.kDrivetrainSim;
      mPigeonIMUSim = mPigeonIMU.getSimCollection();
      mLeftSimcoder = new TalonEncoderSim(mLeftEncoder);
      mRightSimcoder = new TalonEncoderSim(mRightEncoder);
    }
  }

  /**
   * Configues the drivetrain motors
   */
  public void configureMotors() {

    //Reset motors to default
    mFrontLeft.configFactoryDefault();
    mFrontRight.configFactoryDefault();
    mBackLeft.configFactoryDefault();
    mBackRight.configFactoryDefault();

    //Set the back motors to follow the commands of the front
    mBackLeft.follow(mFrontLeft);
    mBackRight.follow(mFrontRight);

    //Set the neutral modes
    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Coast);
    mBackRight.setNeutralMode(NeutralMode.Coast);

    //Makes Green go Forward. Sim is weird so thats what the if statement is for
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

    //Configures encoders to read in meters
    mLeftEncoder.setDistancePerPulse(Constants.kDistancePerPulse);
    mRightEncoder.setDistancePerPulse(Constants.kDistancePerPulse);

    // Encoders
    mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mBackLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mBackRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Limits the current to prevent breaker tripping
    mFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5)); // | Enabled | 60a Limit | 65a Thresh | .5 sec Trigger Time
    mFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5));// | Enabled | 60a Limit | 65a Thresh | .5 sec Trigger Time
    mBackLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5));  // | Enabled | 60a Limit | 65a Thresh | .5 sec Trigger Time
    mBackRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5)); // | Enabled | 60a Limit | 65a Thresh | .5 sec Trigger Time

  }

  /*
   * 
   * Reset Methods
   * 
   */

  /**
   * Resets the odometry
   * @param pose New Position
   */
  public void resetOdometry(Pose2d pose) {
    if(RobotBase.isSimulation()){
      mDrivetrainSim = Constants.kDrivetrainSim;
    }
    resetEncoders();
    resetGyro();
    mDifferentialDrivePoseEstimator.resetPosition(pose, pose.getRotation());
  }

  /**
   * Resets the encoders
   */
  public void resetEncoders(){
    mLeftEncoder.reset();
    mRightEncoder.reset();
  }

  /**
   * Resets the gyro
   */
  public void resetGyro(){
    mPigeonIMU.reset();
  }

  /*
   * 
   * Set Methods
   * 
   */

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = mFeedForward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = mFeedForward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = mPIDController.calculate(mLeftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = mPIDController.calculate(mRightEncoder.getRate(), speeds.rightMetersPerSecond);

    mFrontLeft.setVoltage(leftOutput + leftFeedforward);
    mFrontRight.setVoltage(rightOutput + rightFeedforward);
  }

  public void setSpeeds(WheelSpeeds speeds) {

    speeds.left *= Constants.kMaxSpeed;
    speeds.right *= Constants.kMaxSpeed;

    final double leftFeedforward = mFeedForward.calculate(speeds.left);
    final double rightFeedforward = mFeedForward.calculate(speeds.right);

    final double leftOutput = mPIDController.calculate(mLeftEncoder.getRate(), speeds.left);
    final double rightOutput = mPIDController.calculate(mRightEncoder.getRate(), speeds.right);

    mFrontLeft.setVoltage(leftOutput + leftFeedforward);
    mFrontRight.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Stops the Drivetrain
   */
  public void stop(){
    mFrontLeft.set(0);
    mFrontRight.set(0);
  }

  public void setCheesyDrive(){

  }

  /*
   * Get Methods 
   */

  /**
   * @return Robot Position
   */
  public Pose2d getPose() {
    return mDifferentialDrivePoseEstimator.getEstimatedPosition();
  }

  /**
   * Get left drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "Left Distance")
  public double getLeftMeters() {
    return mLeftEncoder.getDistance();
  }

  /**
   * Get right drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 2, width = 2, height = 1, name = "Right Distance")
  public double getRightMeters() {
    return mRightEncoder.getDistance();
  }

  /**
   * Get left drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 0, width = 2, height = 1, name = "Left Velocity")
  public double getLeftVelocity() {
    return mLeftEncoder.getRate();
  }

  /**
   * Get right drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 2, width = 2, height = 1, name = "Right Velocity")
  public double getRightVelocity() {
    return mRightEncoder.getRate();
  }

  /**
   * Get temperature of the front left motor in celsius
   */
  @Log.Dial(rowIndex = 0, columnIndex = 4, width = 2, height = 2, name = "FL Temp", max = 110, min = 20, showValue = false)
  public double getFrontLeftTemp() {
    if(RobotBase.isReal()){return mFrontLeft.getTemperature();}
    return 0;
  }

  /**
   * Get temperature of the front right motor in celsius
   */
  @Log.Dial(rowIndex = 0, columnIndex = 6, width = 2, height = 2, name = "FR Temp", max = 110, min = 20, showValue = false)
  public double getFrontRightTemp() {
    if(RobotBase.isReal()){return mFrontRight.getTemperature();}
    return 0;
  }

  /**
   * Get temperature of the back left motor in celsius
   */
  @Log.Dial(rowIndex = 2, columnIndex = 4, width = 2, height = 2, name = "BL Temp", max = 110, min = 20, showValue = false)
  public double getBackLeftTemp() {
    if(RobotBase.isReal()){return mBackLeft.getTemperature();}
    return 0;
  }

  /**
   * Get temperature of the back right motor in celsius
   */
  @Log.Dial(rowIndex = 2, columnIndex = 6, width = 2, height = 2, name = "BR Temp", max = 110, min = 20, showValue = false)
  public double getBackRightTemp() {
    if(RobotBase.isReal()){return mBackRight.getTemperature();}
    return 0;
  }

  @Log.Dial(rowIndex = 2, columnIndex = 2, width = 1, height = 1, name = "Left Vel", min = -Constants.kMaxSpeed, max = Constants.kMaxSpeed, showValue = false)
  public double leftVelocityGauge(){
    return getLeftVelocity();
  }

  @Log.Dial(rowIndex = 2, columnIndex = 3, width = 1, height = 1, name = "Right Vel", min = -Constants.kMaxSpeed, max = Constants.kMaxSpeed, showValue = false)
  public double rightVelocityGauge(){
    return getRightVelocity();
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
      setSpeeds(DifferentialDrive.curvatureDriveIK(xSpeed.getAsDouble(), zRotation.getAsDouble(), isQuickturn.getAsBoolean()));
    }

    @Override
    public void end(boolean interrupted) {
      stop();
    }
  }
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
      
      if(resetPose){
        resetOdometry(trajectory.getInitialPose());
      }

      mPathPoints.clear();
      mPathPoints.add(mDifferentialDrivePoseEstimator.getEstimatedPosition());
      mRobotPath.setPoses(mPathPoints);
      mTrajectoryPlot.setTrajectory(trajectory);

      timer.start();
    }

    @Override
    public void execute() {
        mPathPoints.add(mDifferentialDrivePoseEstimator.getEstimatedPosition());
        mRobotPath.setPoses(mPathPoints);

        State desiredPose = trajectory.sample(timer.get());
        ChassisSpeeds refChassisSpeeds = mRamseteController.calculate(mDifferentialDrivePoseEstimator.getEstimatedPosition(), desiredPose);
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

  public class ResetPosition extends InstantCommand{
    @Override
    public void initialize() {
      resetOdometry(new Pose2d());
    }
  }


  /*
   *
   * WPILib Methods
   * 
   */

  @Override
  public void periodic() {
    mDifferentialDrivePoseEstimator.update(
      mPigeonIMU.getRotation2d(),
      new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()),
      getLeftMeters(),
      getRightMeters()
    );

    mField.setRobotPose(mDifferentialDrivePoseEstimator.getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {

    mDrivetrainSim.setInputs(mFrontLeft.get() * RobotController.getInputVoltage(),
                             mFrontRight.get() * RobotController.getInputVoltage());
  
    mDrivetrainSim.update(0.02);

    mLeftSimcoder.setDistance(mDrivetrainSim.getLeftPositionMeters());
    mLeftSimcoder.setRate(mDrivetrainSim.getLeftVelocityMetersPerSecond());

    mRightSimcoder.setDistance(mDrivetrainSim.getRightPositionMeters());
    mRightSimcoder.setRate(mDrivetrainSim.getRightVelocityMetersPerSecond());

    mPigeonIMUSim.setRawHeading(mDrivetrainSim.getHeading().getDegrees());
  }
}
