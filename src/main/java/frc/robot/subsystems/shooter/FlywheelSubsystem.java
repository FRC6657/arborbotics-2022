package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.custom.ArborMath;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class FlywheelSubsystem extends SubsystemBase implements Loggable {

  private final WPI_TalonFX mMaster = new WPI_TalonFX(Constants.CAN.kFlywheelMaster);
  private final WPI_TalonFX mSlave = new WPI_TalonFX(Constants.CAN.kFlywheelSlave);

  private final TalonFXSimCollection mMasterSim = mMaster.getSimCollection();

  private PIDController mPID = new PIDController(20d/9570d, 0, 0);
  private final SimpleMotorFeedforward mFeedForward = Constants.Flywheel.kFeedForward;

  public double mTargetRPM, mCurrentRPM, mPIDEffort, mFFEffort;

  public static final LinearSystem<N1, N1, N1> kPlant = LinearSystemId.identifyVelocitySystem(12d/6380d, 0.12);

  LinearFilter mNoiseFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  VisionSupplier vision;

  public boolean useVision = false;

  private FlywheelSim mSim = new FlywheelSim(
    kPlant,
    DCMotor.getFalcon500(1),
    1/1.5
  );

  public FlywheelSubsystem(VisionSupplier vision) {
    configureMotor();
    mPID.setTolerance(100);
    mTargetRPM = 0;
    this.vision = vision;
  }

  public void configureMotor(){
    mMaster.configFactoryDefault();
    mMaster.setNeutralMode(NeutralMode.Coast);

    mMaster.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    mMaster.configVelocityMeasurementWindow(1);

    mMaster.setInverted(true);

    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 250);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 250);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 250);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 250);
    mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 250);

    mMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));

    mSlave.configFactoryDefault();
    mSlave.setNeutralMode(NeutralMode.Coast); 

    mSlave.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms);
    mSlave.configVelocityMeasurementWindow(32);

    mSlave.setInverted(false);

    mSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 250);
    mSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 250);
    mSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 250);
    mSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 250);
    mSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 250);

    mSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));

  }
  
  public void runFlywheel(){

    mCurrentRPM = getRPM();

    // if(vision.hasTarget() && useVision){
    //   setTargetRPM(InterpolatingTable.get(vision.getDistance()).rpm);
    // }

    if(mTargetRPM != 0){
      mFFEffort = mFeedForward.calculate(mTargetRPM+200);
      mPIDEffort = mPID.calculate(mCurrentRPM, mTargetRPM+200);
    }else{
      mFFEffort = 0;
      mPIDEffort = 0;
    }

    mMaster.setVoltage(mPIDEffort + mFFEffort);

  }

  @Config(tabName = "Shooter", name = "Set RPM")  
  public void setTargetRPM(double newTarget){
    mTargetRPM = newTarget;
  }

  @Log(tabName = "Shooter", name = "Filtered RPM")
  public double getRPM(){
    return mNoiseFilter.calculate((mMaster.getSelectedSensorVelocity()/2048d)*600*1.5);
  }

  @Log(tabName = "Shooter", name ="Raw RPM")
  private double getRawRPM(){
    return (mMaster.getSelectedSensorVelocity()/2048d)*600*1.5;
  }

  @Log(tabName = "Shooter", name = "Flywheel Ready")
  public boolean ready(){
    return ArborMath.inTolerance(Math.abs(mTargetRPM+200-mCurrentRPM), 100) && mTargetRPM != 0;
  }

  public void stop(){
    setTargetRPM(0);
    mMaster.setVoltage(0);
  }

  @Override
  public void periodic() {
    runFlywheel();
  }

  @Override
  public void simulationPeriodic() {

    mSim.setInput(mMaster.get() * RobotController.getInputVoltage());

    mSim.update(0.02);

    double flywheelNativeVelocity = mSim.getAngularVelocityRPM() * 2048d / (60d * 10d) * 1d/1.5d;
    double flywheelNativePositionDelta = flywheelNativeVelocity*10*0.02;

    mMasterSim.setIntegratedSensorVelocity((int)flywheelNativeVelocity);
    mMasterSim.addIntegratedSensorPosition((int)flywheelNativePositionDelta);

    mMasterSim.setBusVoltage(RobotController.getBatteryVoltage());

  }

  @Log(tabName = "Shooter", name = "RPM Target")
  private double getRPMTarget(){
    return mTargetRPM;
  }

  public void enableVision(){
    useVision = true;
  }

  public void disableVision(){
    useVision = false;
  }

}
