// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  private static double kTrackWidth = Units.inchesToMeters(26); //TODO Measure
  private static double kEncoderCountToMeters = 1024 * 2 * Math.PI * Units.inchesToMeters(3);

  //Drivetrain Falcons
  private WPI_TalonFX mFrontLeft = new WPI_TalonFX(1);
  private WPI_TalonFX mFrontRight = new WPI_TalonFX(2);
  private WPI_TalonFX mBackLeft = new WPI_TalonFX(3);
  private WPI_TalonFX mBackRight = new WPI_TalonFX(4);

  //Simulated Talons
  private TalonFXSimCollection mFrontLeftSim = mFrontLeft.getSimCollection();
  private TalonFXSimCollection mFrontRightSim = mFrontRight.getSimCollection();

  //Gyro
  private WPI_PigeonIMU mPigeonIMU = new WPI_PigeonIMU(5);
  
  //Simulated Gyro
  private BasePigeonSimCollection mPigeonIMUSim = mPigeonIMU.getSimCollection();

  //Drivetrain Class
  private DifferentialDrive mDrivetrain = new DifferentialDrive(mFrontLeft, mFrontRight);

  //Drivetrain Kinematics/Odometry
  private DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(kTrackWidth);
  private DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(mPigeonIMU.getRotation2d());

  //Drivetrain PID Controllers
  private final PIDController mLeftPID = new PIDController(1, 0, 0);
  private final PIDController mRightPID = new PIDController(1, 0, 0);

  //Characterization
  private final SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(1,3); //TODO Run Sysid

  //Field to view Odometry
  private Field2d mField = new Field2d();

  /**
   * This sim is no where near perfectly accurate, should be kinda close though
   */
  private DifferentialDrivetrainSim mDrivetrainSim = new DifferentialDrivetrainSim(
    DCMotor.getFalcon500(2), //Motors Per Side
    10.71, //Gearing 10.71:1
    7.5, //MOI. This is not a real value
    30, //Weight is kg. This is not a real value
    Units.inchesToMeters(3), //Wheel Radius in Meters
    kTrackWidth, //Distance between the sides
    null //Measurement deviation
  );

  public DrivetrainSubsystem(){
    configureMotors();
    resetOdometry();
    SmartDashboard.putData(mField);
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
    resetGyro();
    resetEncoders();
    mOdometry.resetPosition(new Pose2d(), mPigeonIMU.getRotation2d());
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

  private void stop(){
    mDrivetrain.stopMotor();
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

    //Reset Encoders
    mFrontLeft.setSelectedSensorPosition(0);
    mFrontRight.setSelectedSensorPosition(0);
    mBackLeft.setSelectedSensorPosition(0);
    mBackRight.setSelectedSensorPosition(0);

    //Limits the current to prevent breaker tripping
    mFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5)); //| Enabled | 40a Limit | 45a Thresh | .5 sec Trigger Time
    mFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5)); //| Enabled | 40a Limit | 45a Thresh | .5 sec Trigger Time
    mBackLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5)); //| Enabled | 40a Limit | 45a Thresh | .5 sec Trigger Time
    mBackRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5)); //| Enabled | 40a Limit | 45a Thresh | .5 sec Trigger Time

  }

  @Override
  public void periodic() {
    
    //Update the Odometry
    mOdometry.update(
      mPigeonIMU.getRotation2d(),
      mFrontLeft.getSelectedSensorPosition() / kEncoderCountToMeters,
      mFrontRight.getSelectedSensorPosition() / kEncoderCountToMeters
    );

    //Send Robot Pose to the Field Visualization
    mField.setRobotPose(mOdometry.getPoseMeters());
  }

  @Override
    public void simulationPeriodic() {

        mDrivetrainSim.setInputs(
          mFrontLeft.get() * RobotController.getInputVoltage(),
          mFrontRight.get() * RobotController.getInputVoltage()
        );

        mDrivetrainSim.update(0.02);
        
        mFrontLeftSim.setIntegratedSensorRawPosition((int)(mDrivetrainSim.getLeftPositionMeters() * kEncoderCountToMeters));
        mFrontRightSim.setIntegratedSensorRawPosition((int)(mDrivetrainSim.getRightPositionMeters() * kEncoderCountToMeters));

        mPigeonIMUSim.setRawHeading(mDrivetrainSim.getHeading().getDegrees());

    }

  /**
   * Commands
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
        mDrivetrain.curvatureDrive(xSpeed.getAsDouble(), zRotation.getAsDouble(), isQuickturn.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
      stop();
    }

  }

}
