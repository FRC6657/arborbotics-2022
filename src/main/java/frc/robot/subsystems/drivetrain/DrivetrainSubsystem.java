// Copyright (c) FIRST and other WPILib contributors.
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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.custom.ctre.LazyTalonFX;
import frc.robot.custom.ctre.SendablePigeonIMU;
import frc.robot.custom.ctre.TalonEncoder;
import frc.robot.custom.ctre.TalonEncoderSim;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DrivetrainSubsystem extends SubsystemBase implements Loggable{

  private double kS = Constants.kS;
  private double kV = Constants.kV;
  private double kA = Constants.kA;

  //Drivetrain Falcons
  private LazyTalonFX mFrontLeft = new LazyTalonFX(Constants.kFrontLeftID);
  private LazyTalonFX mFrontRight = new LazyTalonFX(Constants.kFrontRightID);
  private LazyTalonFX mBackLeft = new LazyTalonFX(Constants.kBackLeftID);
  private LazyTalonFX mBackRight = new LazyTalonFX(Constants.kBackRightID);

  private TalonEncoder mLeftEncoder = new TalonEncoder(mFrontLeft, true);
  private TalonEncoder mRightEncoder = new TalonEncoder(mFrontLeft, false);

  private TalonEncoderSim mLeftSimcoder = new TalonEncoderSim(mLeftEncoder);
  private TalonEncoderSim mRightSimcoder = new TalonEncoderSim(mLeftEncoder);

  //Gyro
  @Log.Gyro(rowIndex = 2, columnIndex = 0, width = 2, height = 2, name = "Gyro")
  private SendablePigeonIMU mPigeonIMU = new SendablePigeonIMU(5);
  
  //Simulated Gyro
  private BasePigeonSimCollection mPigeonIMUSim = mPigeonIMU.getSimCollection();

  //Drivetrain Kinematics/Odometry
  private DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(Constants.kTrackWidth);
  private DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(mPigeonIMU.getRotation2d());

  //Drivetrain PID Controllers
  private final PIDController mLeftPID = new PIDController(Constants.leftKP, Constants.leftKI, Constants.leftKD);
  private final PIDController mRightPID = new PIDController(Constants.rightKP, Constants.rightKI, Constants.rightKD);

  //Characterization
  private SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  
  //RemseteController
  private RamseteController mRamseteController = new RamseteController();

  //Field to view Odometry
  private Field2d mField = new Field2d();

  //Creates an Object to set as a trajectory later
  private FieldObject2d mTrajectoryPlot = mField.getObject("trajectory");

  //Creates Objects to plot the robot's path
  private FieldObject2d mRobotPath = mField.getObject("robot-path");
  private List<Pose2d> mPathPoints = new ArrayList<Pose2d>();

  /**
   * This sim is no where near perfectly accurate, should be kinda close though
   */
  private DifferentialDrivetrainSim mDrivetrainSim = new DifferentialDrivetrainSim(
    DCMotor.getFalcon500(2), //Motors Per Side
    Constants.kGearRatio, //Gearing 10.71:1
    7.5, //MOI. This is not a real value
    Constants.kRobotWeight, //Weight is kg. This is not a real value
    Constants.kWheelRadius, //Wheel Radius in Meters
    Constants.kTrackWidth, //Distance between the sides
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
  );

  public DrivetrainSubsystem(){
    configureMotors(); //Configure Motors
    resetOdometry(); //Reset Odometry
    SmartDashboard.putData(mField); //Sends the Field to Shuffleboard
  }


  /**
   * 
   * Methods
   * 
   */

  /**
   * Resets the Odometry
   */
  private void resetOdometry() {
    
    if (RobotBase.isSimulation()) {
      mDrivetrainSim = new DifferentialDrivetrainSim(
          DCMotor.getFalcon500(2), // Motors Per Side
          Constants.kGearRatio, // Gearing 10.71:1
          7.5, // MOI. This is not a real value
          Constants.kRobotWeight, // Weight is kg. This is not a real value
          Constants.kWheelRadius, // Wheel Radius in Meters
          Constants.kTrackWidth, // Distance between the sides
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );
    }

    resetGyro();
    resetEncoders();
    mOdometry.resetPosition(new Pose2d(), mPigeonIMU.getRotation2d());
  }

  /**
   * Resets the Odometry
   */
  private void resetOdometry(Pose2d pose) {
    
    if (RobotBase.isSimulation()) {
      mDrivetrainSim = new DifferentialDrivetrainSim(
          DCMotor.getFalcon500(2), // Motors Per Side
          Constants.kGearRatio, // Gearing 10.71:1
          7.5, // MOI. This is not a real value
          Constants.kRobotWeight, // Weight is kg. This is not a real value
          Constants.kWheelRadius, // Wheel Radius in Meters
          Constants.kTrackWidth, // Distance between the sides
          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
      );
    }

    resetGyro();
    resetEncoders();
    mOdometry.resetPosition(pose, mPigeonIMU.getRotation2d());
  }


  /**
   * Resets the Gyro
   */
  private void resetGyro() {
    mPigeonIMU.reset();
  }

  /**
   * Resets the Encoder Positions
   */
  private void resetEncoders(){
    mLeftEncoder.reset();
    mRightEncoder.reset();
  }

  /**
   * Stops all Drivetrain Motors
   */
  private void stop(){
    mFrontLeft.set(0);
    mFrontRight.set(0);
  }

  public void setSpeeds(WheelSpeeds speeds) {

    //Scale input to a Max Speed
    speeds.left *= Constants.kMaxSpeed;
    speeds.right *= Constants.kMaxSpeed;

    final double leftFeedforward = mFeedforward.calculate(speeds.left);
    final double rightFeedforward = mFeedforward.calculate(speeds.right);
    final double leftOutput =
        mLeftPID.calculate(getLeftVelocity(), speeds.left);
    final double rightOutput =
        mRightPID.calculate(getRightVelocity(), speeds.right);

    mFrontLeft.setVoltage(leftOutput + leftFeedforward);
    mFrontRight.setVoltage(rightOutput + rightFeedforward);

  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {

    final double leftFeedforward = mFeedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = mFeedforward.calculate(speeds.rightMetersPerSecond);
    final double leftOutput =
        mLeftPID.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        mRightPID.calculate(getRightVelocity(), speeds.rightMetersPerSecond);

    mFrontLeft.setVoltage(leftOutput + leftFeedforward);
    mFrontRight.setVoltage(rightOutput + rightFeedforward);

  }

  /**
   * Configures all of the Motors
   */
  private void configureMotors() {

    //Reset all of the motors to default values
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
    mBackLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);

    //Makes all positive signals move the robot forward
    if(RobotBase.isReal()){  
      mFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);
      mBackLeft.setInverted(TalonFXInvertType.FollowMaster);
      mFrontRight.setInverted(TalonFXInvertType.Clockwise);
      mBackRight.setInverted(TalonFXInvertType.FollowMaster);
    }else{
      mFrontLeft.setInverted(TalonFXInvertType.CounterClockwise);
      mBackLeft.setInverted(TalonFXInvertType.FollowMaster);
      mFrontRight.setInverted(TalonFXInvertType.CounterClockwise);
      mBackRight.setInverted(TalonFXInvertType.FollowMaster);
    }

    mLeftEncoder.setDistancePerPulse(Constants.kEncoderCountToMeters);
    mRightEncoder.setDistancePerPulse(Constants.kEncoderCountToMeters);

    //Encoders 
    mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mBackLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mBackRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    //Limits the current to prevent breaker tripping
    mFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5)); //| Enabled | 60a Limit | 65a Thresh | .5 sec Trigger Time
    mFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5)); //| Enabled | 60a Limit | 65a Thresh | .5 sec Trigger Time
    mBackLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5)); //| Enabled | 60a Limit | 65a Thresh | .5 sec Trigger Time
    mBackRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.5)); //| Enabled | 60a Limit | 65a Thresh | .5 sec Trigger Time

  }

  /**
   * Get left drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "Left Distance")
  public double getLeftMeters(){
    return mLeftEncoder.getDistance();
  }

  /**
   * Get right drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 2, width = 2, height = 1, name = "Right Distance")
  public double getRightMeters(){
    return mRightEncoder.getDistance();
  }

  /**
   * Get left drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 0, width = 2, height = 1, name = "Left Velocity")
  public double getLeftVelocity(){
    return mLeftEncoder.getRate();
  }

  /**
   * Get right drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 2, width = 2, height = 1, name = "Right Velocity")
  public double getRightVelocity(){
    return mRightEncoder.getRate();
  }

  /**
   * Get temperature of the front left motor in celsius
   */
  @Log.Dial(rowIndex = 0, columnIndex = 4, width = 2, height = 2, name = "FL Temp", max = 110, min = 20, showValue = false)
  public double getFrontLeftTemp(){
    return mFrontLeft.getTemperature();
  }

  /**
   * Get temperature of the front right motor in celsius
   */
  @Log.Dial(rowIndex = 0, columnIndex = 6, width = 2, height = 2, name = "FR Temp", max = 110, min = 20, showValue = false)
  public double getFrontRightTemp(){
    return mFrontRight.getTemperature();
  }

  /**
   * Get temperature of the back left motor in celsius
   */
  @Log.Dial(rowIndex = 2, columnIndex = 4, width = 2, height = 2, name = "BL Temp", max = 110, min = 20, showValue = false)
  public double getBackLeftTemp(){
    return mBackLeft.getTemperature();
  }

  /**
   * Get temperature of the back right motor in celsius
   */
  @Log.Dial(rowIndex = 2, columnIndex = 6, width = 2, height = 2, name = "BR Temp", max = 110, min = 20, showValue = false)
  public double getBackRightTemp(){
    return mBackRight.getTemperature();
  }

  @Override
  public void periodic() {
    
    //Update the Odometry
    mOdometry.update(
      mPigeonIMU.getRotation2d(),
      mLeftEncoder.getDistance(),
      mRightEncoder.getDistance()
    );

    //Send Robot Pose to the Field Visualization
    mField.setRobotPose(mOdometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {

    //Give the sim motor inputs
    mDrivetrainSim.setInputs(
        mFrontLeft.get() * RobotController.getInputVoltage(),
        mFrontRight.get() * RobotController.getInputVoltage());

    //Progress the sim by 1 frame
    mDrivetrainSim.update(0.02);

    //Set Simulated Encoder Positions
    mLeftSimcoder.setDistance(mDrivetrainSim.getLeftPositionMeters());
    mRightSimcoder.setDistance(mDrivetrainSim.getRightPositionMeters());

    //Set Simulated Encoder Velocities
    mLeftSimcoder.setRate(mDrivetrainSim.getLeftVelocityMetersPerSecond());
    mRightSimcoder.setRate(mDrivetrainSim.getRightVelocityMetersPerSecond());

    //Set Simulated Gyro Position
    mPigeonIMUSim.setRawHeading(mDrivetrainSim.getHeading().getDegrees());

  }

  /**
   * Commands
   */

   /**
    * Command for TeleOp Driving
    */
  public class DriveCommand extends CommandBase{
    
    private DoubleSupplier xSpeed;
    private DoubleSupplier zRotation;
    private BooleanSupplier isQuickturn;

    public DriveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier isQuickturn){
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

  /**
   * Command to follow a pregenerated trajectory
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

      if(resetPose){
        resetOdometry(trajectory.getInitialPose());
      }

      mPathPoints.clear();
      mPathPoints.add(mOdometry.getPoseMeters());
      mRobotPath.setPoses(mPathPoints);
      mTrajectoryPlot.setTrajectory(trajectory);

      timer.start();
    }

    @Override
    public void execute() {
      if (timer.get() < trajectory.getTotalTimeSeconds()) {

        mPathPoints.add(mOdometry.getPoseMeters());

        mRobotPath.setPoses(mPathPoints);

        var desiredPose = trajectory.sample(timer.get());

        var refChassisSpeeds = mRamseteController.calculate(mOdometry.getPoseMeters(), desiredPose);

        setSpeeds(mKinematics.toWheelSpeeds(new ChassisSpeeds(refChassisSpeeds.vxMetersPerSecond, 0.0, refChassisSpeeds.omegaRadiansPerSecond)));

      } else {
        stop();
      }
    }

    @Override
    public void end(boolean interrupted) {
      stop();
    }

  }

  /**
   * Command to reset Odometry
   */
  public class ResetPosition extends CommandBase{

    public ResetPosition(){
      addRequirements(DrivetrainSubsystem.this);
    }

    @Override
    public void initialize() {
      resetOdometry();
    }

  }

}
