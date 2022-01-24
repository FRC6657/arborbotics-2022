// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
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
      VecBuilder.fill(8.0), // Velocity error tolerance
      VecBuilder.fill(12.0), // Control effort (voltage) tolerance
      0.020);

  private final LinearSystemLoop<N1, N1, N1> mFlywheelLoop = new LinearSystemLoop<>(Constants.Flywheel.kPlant,
      mFlywheelController, mFlywheelObserver, 12.0, 0.020);

  private final WPI_TalonFX mProtagonist; //, mAntagonist;

  private TalonFXSimCollection mMotorSim;
  private FlywheelSim mFlywheelSim;

  private boolean mAtTarget = false;
  private double mRpmTarget = 0;

  public FlywheelSubsystem() {
    mProtagonist = new WPI_TalonFX(Constants.kLeftFlywheelID);
    /* mAntagonist = new WPI_TalonFX(Constants.kRightFlywheelID); */
    configureMotors();

    if(RobotBase.isSimulation()){
      mMotorSim = mProtagonist.getSimCollection();
      mFlywheelSim = Constants.Flywheel.kSim;
    }

  }

  @Config(rowIndex = 2, columnIndex = 0, width = 2, height = 1, name = "Motor Percent", defaultValueNumeric = 0)
  private void set(double percent) {
    mProtagonist.set(percent);
  }

  public void configureMotors() {
    mProtagonist.configFactoryDefault();
    mProtagonist.setInverted(true);
    mProtagonist.setNeutralMode(NeutralMode.Coast);

    /*
     * mAntagonist.follow(mProtagonist);
     * mAntagonist.setInverted(InvertType.OpposeMaster);
     * mAntagonist.setNeutralMode(NeutralMode.Coast);
     */

     mProtagonist.setSelectedSensorPosition(0);

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

  public class setRPMTarget extends CommandBase{
    double newTarget;
    public setRPMTarget(double newTarget){
      this.newTarget = newTarget;
      addRequirements(FlywheelSubsystem.this);
    }
    @Override
    public void initialize() {
      mRpmTarget = newTarget;
    }
    @Override
    public boolean isFinished() {
        return true;
    }
  }

  public class WaitForFlywheel extends CommandBase{
    public WaitForFlywheel(){
      addRequirements(FlywheelSubsystem.this);
    }
    @Override
    public boolean isFinished() {
        return atTarget();
    }
  }

  public class FlywheelController extends CommandBase {

    public FlywheelController() {
      addRequirements(FlywheelSubsystem.this);
    }

    @Override
    public void execute() {

      if(getRPMDelta() < Constants.Flywheel.kRPMTollerance){
        mAtTarget = true;
      }
      else{
        mAtTarget = false;
      }

      mFlywheelLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(mRpmTarget)));
      mFlywheelLoop.correct(VecBuilder.fill(getRadiansPerSecond()));
      mFlywheelLoop.predict(0.020);
      double mNextVolts = mFlywheelLoop.getU(0);
      mProtagonist.setVoltage(mRpmTarget == 0 ? 0 : mNextVolts);
    }
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

    SmartDashboard.putNumber("simRPM", mFlywheelSim.getAngularVelocityRPM());

  }
}
