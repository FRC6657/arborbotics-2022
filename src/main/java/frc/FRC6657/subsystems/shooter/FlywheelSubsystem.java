// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.ArborMath;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class FlywheelSubsystem extends SubsystemBase implements Loggable {

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> mFlywheelObserver = new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      Constants.Flywheel.kPlant,
      VecBuilder.fill(3.0), // How accurate we think our model is
      VecBuilder.fill(0.01), // How accurate we think our encoder data is
      0.020);

  private final LinearQuadraticRegulator<N1, N1, N1> mFlywheelController = new LinearQuadraticRegulator<>(
      Constants.Flywheel.kPlant,
      VecBuilder.fill(500.0), // Velocity error tolerance
      VecBuilder.fill(12.0), // Control effort (voltage) tolerance
      0.020);

  private final LinearSystemLoop<N1, N1, N1> mFlywheelLoop = new LinearSystemLoop<>(Constants.Flywheel.kPlant,
      mFlywheelController, mFlywheelObserver, 12.0, 0.020);

  private final WPI_TalonFX mProtagonist, mAntagonist;

  private TalonFXSimCollection mMotorSim;
  private FlywheelSim mFlywheelSim;

  private boolean mAtTarget = false;
  private double mRpmTarget = 0;

  public FlywheelSubsystem() {
    mProtagonist = new WPI_TalonFX(Constants.kLeftFlywheelID);
    mAntagonist = new WPI_TalonFX(Constants.kRightFlywheelID);

    configureMotors();

    if(RobotBase.isSimulation()){
      mMotorSim = mProtagonist.getSimCollection();
      mFlywheelSim = Constants.Flywheel.kSim;
    }

  }

  public void stop(){
    setRPMTarget(0);
    mProtagonist.stopMotor();
  }

  @Log(rowIndex = 2, columnIndex = 2, width = 2, height = 1, name = "Flywheel Percent")
  public double getMotorPercent(){
    return mProtagonist.get();
  }

  public void configureMotors() {

    mProtagonist.configFactoryDefault();
    mAntagonist.configFactoryDefault();

    mAntagonist.follow(mProtagonist);

    mProtagonist.setNeutralMode(NeutralMode.Coast);
    mAntagonist.setNeutralMode(NeutralMode.Coast);

    mProtagonist.setInverted(InvertType.None);
    mAntagonist.setInverted(InvertType.OpposeMaster);

    mProtagonist.setSelectedSensorPosition(0);
    mAntagonist.setSelectedSensorPosition(0);

    mProtagonist.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    mProtagonist.configVelocityMeasurementWindow(1);

    mAntagonist.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    mAntagonist.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

  }

  public double getRadiansPerSecond() {
    return (mProtagonist.getSelectedSensorVelocity() * 10) * (2.0 * Math.PI / Constants.kFalconEncoderCPR / Constants.Flywheel.kRatio);
  }

  @Log(rowIndex = 1, columnIndex = 0, width = 2, height = 1, name = "RPM")
  public double getRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(getRadiansPerSecond());
  }

  @Log(rowIndex = 0, columnIndex = 2, width = 2, height = 1, name = "RPM Target")
  public double getRPMTarget(){
    return mRpmTarget;
  }

  @Log(rowIndex = 1, columnIndex = 2, width = 2, height = 1, name = "RPM Delta")
  public double getRPMDelta(){
    return Math.abs(getRPMTarget()-getRPM());
  }

  @Log(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "At RPM Target")
  public boolean atTarget(){
    return mAtTarget;
  }

  public boolean active(){
    return mRpmTarget != 0;
  }

  @Config(rowIndex = 3, columnIndex = 0, width = 2, height = 1, name="Set RPM Target", defaultValueNumeric = 0)
  public void setRPMTarget(double setpoint){
    mRpmTarget = setpoint;
  }

  public void set(double percent){
    mProtagonist.set(percent);
  }

  @Override
  public void periodic() {
    
    mAtTarget = ArborMath.inTolerance(getRPMDelta(), Constants.Flywheel.kRPMTollerance) && getRPMTarget() !=0;

    mFlywheelLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(mRpmTarget)));
    mFlywheelLoop.correct(VecBuilder.fill(getRadiansPerSecond()));
    mFlywheelLoop.predict(0.020);
    double mNextVolts = mFlywheelLoop.getU(0);
    mProtagonist.setVoltage(mRpmTarget == 0 ? 0 : mNextVolts);
  }

  @Override
  public void simulationPeriodic() {

    mFlywheelSim.setInput(mProtagonist.get() * RobotController.getInputVoltage());

    mFlywheelSim.update(0.02);

    double flywheelNativeVelocity = mFlywheelSim.getAngularVelocityRPM() * 2048 / (60 * 10) * Constants.Flywheel.kRatio;
    double flywheelNativePositionDelta = flywheelNativeVelocity*10*0.02;

    mMotorSim.setIntegratedSensorVelocity((int)flywheelNativeVelocity);
    mMotorSim.addIntegratedSensorPosition((int)flywheelNativePositionDelta);

    mMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

  }
}
