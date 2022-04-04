
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_TalonFX mFrontLeft = new WPI_TalonFX(Constants.CAN.kDrive_FrontLeft);
  private final WPI_TalonFX mFrontRight = new WPI_TalonFX(Constants.CAN.kDrive_FrontRight);
  private final WPI_TalonFX mBackLeft = new WPI_TalonFX(Constants.CAN.kDrive_BackLeft);
  private final WPI_TalonFX mBackRight = new WPI_TalonFX(Constants.CAN.kDrive_BackRight);

  private final TalonFXSimCollection mLeftSim = mFrontLeft.getSimCollection();
  private final TalonFXSimCollection mRightSim = mFrontRight.getSimCollection();

  private final WPI_Pigeon2 mPigeon = new WPI_Pigeon2(Constants.CAN.kPigeon);

  private final BasePigeonSimCollection mPigeonSim = mPigeon.getSimCollection();

  private final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.kTrackwidth);
  private final DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(mPigeon.getRotation2d());

  private PIDController mPID = new PIDController(0.5, 0, 0);
  private RamseteController mRamseteController = new RamseteController();

  public static final LinearSystem<N2,N2,N2> mDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
    2.2195, //Linear kV
    0.32787, //Linear kA
    2.53, //Angular kV
    0.081887 //Angular kA
  );

  private LinearPlantInversionFeedforward<N2,N2,N2> mFeedForward = new LinearPlantInversionFeedforward<N2,N2,N2>(mDrivetrainPlant,0.02);

  // Default Sim
  public DifferentialDrivetrainSim mDrivetrainSim = new DifferentialDrivetrainSim( // Simulation
      mDrivetrainPlant,
      DCMotor.getFalcon500(2),
      KitbotGearing.k10p71.value,
      Constants.Drivetrain.kTrackwidth,
      KitbotWheelSize.kSixInch.value,
      null
  );

  private FieldObject2d mTrajectoryPlot = RobotContainer.getField().getObject("trajectory");
  private FieldObject2d mRobotPath = RobotContainer.getField().getObject("robot-path");
  private List<Pose2d> mPathPoints = new ArrayList<Pose2d>();

  VisionSupplier vision;

  public DrivetrainSubsystem(VisionSupplier vision) {
    configureMotors();
    this.vision = vision;
  }

  public void configureMotors(){

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
 
     mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
     mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

     mFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
     mFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
     mBackLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
     mBackRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));

     mFrontLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
     mFrontLeft.configVelocityMeasurementWindow(1);
 
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 227);
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 229);
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239);
     mFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 241);
 
     mFrontRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
     mFrontRight.configVelocityMeasurementWindow(1);
 
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 227);
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 229);
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239);
     mFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 241);
 
     mBackLeft.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
     mBackLeft.configVelocityMeasurementWindow(32);
 
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 227);
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 229);
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239);
     mBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 241);
 
     mBackRight.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
     mBackRight.configVelocityMeasurementWindow(32);
 
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 227);
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 229);
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 233);
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 239);
     mBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 241);

  }



  //
  //
  //  Get Methods
  //
  //

  public DifferentialDriveWheelSpeeds getWheelVelocities(){
    return new DifferentialDriveWheelSpeeds(
      mFrontLeft.getSelectedSensorVelocity() * 10 * Constants.Drivetrain.kDistancePerPulse,
      mFrontRight.getSelectedSensorVelocity() * 10 * Constants.Drivetrain.kDistancePerPulse
    );
  }

  public double[] getWheelDistances(){
    double[] distances = {
      mFrontLeft.getSelectedSensorPosition() * Constants.Drivetrain.kDistancePerPulse,
      mFrontRight.getSelectedSensorPosition() * Constants.Drivetrain.kDistancePerPulse
    };
    return distances;
  }

  public Pose2d getRobotPosition(){
    return mOdometry.getPoseMeters();
  }

  public double getHeading(){
    return -mPigeon.getYaw();
  }


  //
  //
  //  Set Methods
  //
  //

  public void stop(){
    mFrontLeft.set(0);
    mFrontRight.set(0);
  }

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

    final double leftOutput = mPID.calculate(getWheelVelocities().leftMetersPerSecond, speeds.leftMetersPerSecond);
    final double rightOutput = mPID.calculate(getWheelVelocities().rightMetersPerSecond, speeds.rightMetersPerSecond);
    
    mFrontLeft.setVoltage(leftOutput + feedForward.get(0,0));
    mFrontRight.setVoltage(rightOutput + feedForward.get(1,0));

  }

  public void resetEncoders(){
    mFrontLeft.setSelectedSensorPosition(0);
    mFrontRight.setSelectedSensorPosition(0);
  }

  public void resetOdometry(){
    resetOdometry(new Pose2d());
  }
  public void resetOdometry(Pose2d newPose){
    resetEncoders();
    mPathPoints.clear();
    mOdometry.resetPosition(newPose, mPigeon.getRotation2d());
  }

  public void resetSimulation(){
    mDrivetrainSim = new DifferentialDrivetrainSim( // Simulation
      mDrivetrainPlant,
      DCMotor.getFalcon500(2),
      KitbotGearing.k10p71.value,
      Constants.Drivetrain.kTrackwidth,
      KitbotWheelSize.kSixInch.value,
      null
    );
  }

  //
  // Periodics
  //

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

  }

  @Override
  public void periodic() {
    mOdometry.update(mPigeon.getRotation2d(), getWheelDistances()[0], getWheelDistances()[1]);

    // Pose2d pose = PhotonUtils.estimateFieldToRobot(
    //   PhotonUtils.estimateCameraToTarget(
    //     PhotonUtils.estimateCameraToTargetTranslation(
    //       vision.getDistance(),
    //       Rotation2d.fromDegrees(vision.getYaw())
    //     ),
    //     Constants.Vision.kTargetPos,
    //     mPigeon.getRotation2d()
    //   ),
    //   Constants.Vision.kTargetPos,
    //   Constants.Vision.kCameraToRobot
    // );

  }

  public class TeleOpCommand extends CommandBase {
    private final DoubleSupplier xInput;
    private final DoubleSupplier zInput;
    private final BooleanSupplier modSpeed;

    // private SlewRateLimiter mForwardAccelLimit = new SlewRateLimiter(Constants.DriveProfile.kDriveForwardAccel);
    // private SlewRateLimiter mForwardDecelLimit = new SlewRateLimiter(Constants.DriveProfile.kDriveForwardDecel);
    // private SlewRateLimiter mBackwardAccelLimit = new SlewRateLimiter(Constants.DriveProfile.kDriveBackwardAccel);
    // private SlewRateLimiter mBackwardDecelLimit = new SlewRateLimiter(Constants.DriveProfile.kDriveBackwardDecel);
    // private SlewRateLimiter mTurnAccelLimit = new SlewRateLimiter(Constants.DriveProfile.kTurnAccel);

    public TeleOpCommand(DoubleSupplier xInput, DoubleSupplier zInput, BooleanSupplier modSpeed){
      this.xInput = xInput;
      this.zInput = zInput;
      this.modSpeed = modSpeed;
      addRequirements(DrivetrainSubsystem.this);
    }

    @Override
    public void execute() {

      double speedMod = modSpeed.getAsBoolean() == false ? Constants.DriveProfile.kMaxDriveSpeed : Constants.DriveProfile.kModDriveSpeed;

      var speeds = 
        new ChassisSpeeds(
          xInput.getAsDouble() * speedMod,
          0,
          zInput.getAsDouble() * Constants.DriveProfile.kMaxTurnSpeed
      );

      setSpeeds(mKinematics.toWheelSpeeds(speeds));

    }

  }

  public class TrajectoryFollowerCommand extends CommandBase {

    private final Timer timer = new Timer();
    private final Trajectory trajectory;

    public TrajectoryFollowerCommand(Trajectory trajectory) {
      this.trajectory = trajectory;
      addRequirements(DrivetrainSubsystem.this);
    }

    @Override
    public void initialize() {

      mPathPoints.add(getRobotPosition());
      mRobotPath.setPoses(mPathPoints);
      mTrajectoryPlot.setTrajectory(trajectory);

      timer.reset();
      timer.start();
    }

    @Override
    public void execute() {

        mPathPoints.add(getRobotPosition());
        mRobotPath.setPoses(mPathPoints);

        State desiredPose = trajectory.sample(timer.get());
        ChassisSpeeds refChassisSpeeds = mRamseteController.calculate(getRobotPosition(), desiredPose);
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

  public class TurnByAngleCommand extends CommandBase {

    double mSetpoint;
    double mStartPoint;

    final PIDController mPIDController = new PIDController(1/2d, 0, 1d/3000);

   /**
    * Turn a specified Degree Amount
    */
    public TurnByAngleCommand(double mSetpoint) {
      this.mSetpoint = mSetpoint;
    }

    @Override
    public void initialize() {
      this.mStartPoint = mPigeon.getYaw();
    }

    @Override
    public void execute() {
      double output = mPIDController.calculate(mPigeon.getYaw(), mStartPoint + mSetpoint);
      mFrontLeft.setVoltage(-output*12);
      mFrontRight.setVoltage(output*12);

    }

    @Override
    public boolean isFinished() {
      return Math.abs((mStartPoint + mSetpoint) - mPigeon.getYaw()) <= .5;
    }

  }

  public class VisionAimAssist extends CommandBase{
    
    PIDController mVisionPID = new PIDController(0.015, 0, 0.001);

    double startPoint;
    MedianFilter filter = new MedianFilter(2);

    public VisionAimAssist(){
      //mVisionPID.setTolerance(0.5, 1);
    }

    @Override
    public void initialize() {
      startPoint = mPigeon.getYaw();
    }

    @Override
    public void execute() {

        double effort = mVisionPID.calculate(vision.getYaw(), 0);

        mFrontLeft.setVoltage(effort * 12);
        mFrontRight.setVoltage(-effort * 12);

    }
    @Override
    public void end(boolean interrupted) {
        stop();
    }

    @Override
    public boolean isFinished() {
      //return Math.abs(vision.getYaw()) < 1;
      return false;
    }

  }

}
