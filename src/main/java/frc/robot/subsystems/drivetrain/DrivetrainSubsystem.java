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
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
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
import frc.robot.custom.SendablePigeonIMU;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DrivetrainSubsystem extends SubsystemBase implements Loggable{

  private double kS = Constants.kS;
  private double kV = Constants.kV;

  //Drivetrain Falcons
  private final WPI_TalonFX mFrontLeft = new WPI_TalonFX(Constants.kFrontLeftID);
  private final WPI_TalonFX mFrontRight = new WPI_TalonFX(Constants.kFrontRightID);
  private final WPI_TalonFX mBackLeft = new WPI_TalonFX(Constants.kBackLeftID);
  private final WPI_TalonFX mBackRight = new WPI_TalonFX(Constants.kBackRightID);

  //Simulated Talons
  private TalonFXSimCollection mFrontLeftSim = mFrontLeft.getSimCollection();
  private TalonFXSimCollection mFrontRightSim = mFrontRight.getSimCollection();

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
  private SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(kS, kV); //TODO Run Sysid
  
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
    Units.lbsToKilograms(45), //Weight is kg. This is not a real value
    Constants.kWheelRadius, //Wheel Radius in Meters
    Constants.kTrackWidth, //Distance between the sides
    null //Measurement deviation
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
          Units.lbsToKilograms(45), // Weight is kg. This is not a real value
          Constants.kWheelRadius, // Wheel Radius in Meters
          Constants.kTrackWidth, // Distance between the sides
          null // Measurement deviation
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
          Units.lbsToKilograms(45), // Weight is kg. This is not a real value
          Constants.kWheelRadius, // Wheel Radius in Meters
          Constants.kTrackWidth, // Distance between the sides
          null // Measurement deviation
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
    mFrontLeft.setSelectedSensorPosition(0);
    mFrontRight.setSelectedSensorPosition(0);
    mBackLeft.setSelectedSensorPosition(0);
    mBackRight.setSelectedSensorPosition(0);
  }

  /**
   * Stops all Drivetrain Motors
   */
  private void stop(){
    mFrontLeft.set(0);
    mFrontRight.set(0);
  }

  /**
   * Sets motor voltages based on input target velocities in meters/s
   * 
   * @param speeds Input speeds Meters/s
   * 
   */
  public void setSpeeds(WheelSpeeds speeds) {

    //Scale input to a Max Speed
    speeds.left *= Constants.kMaxSpeed;
    speeds.right *= Constants.kMaxSpeed;

    final double leftFeedforward = mFeedforward.calculate(speeds.left);
    final double rightFeedforward = mFeedforward.calculate(speeds.right);
    final double leftOutput =
        mLeftPID.calculate(getEncoderRate()[0], speeds.left);
    final double rightOutput =
        mRightPID.calculate(getEncoderRate()[1], speeds.right);

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
   * Returns a double array with left and right in m/s²
   */
  public double[] getEncoderRate(){
    double[] rates = {getLeftVelocity(),getRightVelocity()};
    return rates;
  }

  /**
   * Get left drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "Left Distance")
  public double getLeftMeters(){
    return mFrontLeft.getSelectedSensorPosition() * Constants.kEncoderCountToMeters;
  }

  /**
   * Get right drivetrain encoder distance in meters
   */
  @Log(rowIndex = 0, columnIndex = 2, width = 2, height = 1, name = "Right Distance")
  public double getRightMeters(){
    return mFrontRight.getSelectedSensorPosition() * Constants.kEncoderCountToMeters;
  }

  /**
   * Get left drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 0, width = 2, height = 1, name = "Left Velocity")
  public double getLeftVelocity(){
    return mFrontLeft.getSelectedSensorVelocity() * Constants.kEncoderCountToMeters * 10;
  }

  /**
   * Get right drivetrain motor velocity in m/s²
   */
  @Log(rowIndex = 1, columnIndex = 2, width = 2, height = 1, name = "Right Velocity")
  public double getRightVelocity(){
    return mFrontLeft.getSelectedSensorVelocity() * Constants.kEncoderCountToMeters * 10;
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

  /**
   * Change kV
   * @param KV new kV
   */
  @Config(rowIndex = 2, columnIndex = 2, width = 2, height = 1, name = "kV", defaultValueNumeric = Constants.kV)
  public void setKV(double KV){
    kV = KV;
    mFeedforward = new SimpleMotorFeedforward(kS, kV);
  }

  /**
   * Change kS
   * @param KS new kS
   */
  @Config(rowIndex = 3, columnIndex = 2, width = 2, height = 1, name = "kS", defaultValueNumeric = Constants.kS)
  public void setKS(double KS){
    kS = KS;
    mFeedforward = new SimpleMotorFeedforward(kS, kV);
  }

  /**
   * Converts DifferentialDriveWheelSpeeds to DifferentialDrive.WheelSpeeds I have no idea why they are separated
   * 
   * @param diffSpeeds DifferentialDriveWheelSpeeds object
   */
  public WheelSpeeds convertSpeeds(DifferentialDriveWheelSpeeds diffSpeeds){
    return new WheelSpeeds(diffSpeeds.leftMetersPerSecond,diffSpeeds.rightMetersPerSecond);
  }

  @Override
  public void periodic() {
    
    //Update the Odometry
    mOdometry.update(
      mPigeonIMU.getRotation2d(),
      mFrontLeft.getSelectedSensorPosition() * Constants.kEncoderCountToMeters,
      mFrontRight.getSelectedSensorPosition() * Constants.kEncoderCountToMeters
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
    mFrontLeftSim.setIntegratedSensorRawPosition((int) (mDrivetrainSim.getLeftPositionMeters() / Constants.kEncoderCountToMeters));
    mFrontRightSim.setIntegratedSensorRawPosition((int) (mDrivetrainSim.getRightPositionMeters() / Constants.kEncoderCountToMeters));

    //Set Simulated Encoder Velocities
    mFrontLeftSim.setIntegratedSensorVelocity((int) ((mDrivetrainSim.getLeftVelocityMetersPerSecond() / Constants.kEncoderCountToMeters) / 10));
    mFrontRightSim.setIntegratedSensorVelocity((int) ((mDrivetrainSim.getRightVelocityMetersPerSecond() / Constants.kEncoderCountToMeters) / 10));

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

        setSpeeds(convertSpeeds(mKinematics.toWheelSpeeds(new ChassisSpeeds(refChassisSpeeds.vxMetersPerSecond, 0.0, refChassisSpeeds.omegaRadiansPerSecond))));

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
