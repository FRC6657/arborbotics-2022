// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSubsystem extends SubsystemBase {

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> mFlywheelObserver = new KalmanFilter<>(
      Nat.N1(),
      Nat.N1(),
      Constants.kFlywheelPlant,
      VecBuilder.fill(3.0), // How accurate we think our model is
      VecBuilder.fill(0.01), // How accurate we think our encoder data is
      0.020);

  private final LinearQuadraticRegulator<N1, N1, N1> mFlywheelController = new LinearQuadraticRegulator<>(
      Constants.kFlywheelPlant,
      VecBuilder.fill(8.0), // Velocity error tolerance
      VecBuilder.fill(12.0), // Control effort (voltage) tolerance
      0.020);

  private final LinearSystemLoop<N1, N1, N1> mFlywheelLoop =
      new LinearSystemLoop<>(Constants.kFlywheelPlant, mFlywheelController, mFlywheelObserver, 12.0, 0.020);

  private WPI_TalonFX mProtagonist, mAntagonist;


  public FlywheelSubsystem() {
    mProtagonist = new WPI_TalonFX(Constants.kLeftFlywheelID);
    mAntagonist = new WPI_TalonFX(Constants.kRightFlywheelID);
    configureMotors();

  }

  public void configureMotors(){
    mProtagonist.setInverted(false);
    mProtagonist.setNeutralMode(NeutralMode.Coast);

    mAntagonist.follow(mProtagonist);
    mAntagonist.setInverted(InvertType.OpposeMaster);
    mAntagonist.setNeutralMode(NeutralMode.Coast);

  }

  public double getRadiansPerSecond(){
    return (mProtagonist.getSelectedSensorVelocity() * 10) * (2.0 * Math.PI / Constants.kEncoderCountToMeters);
  }

  public class AdjustRPM extends CommandBase {

    private double mRPM;

    public AdjustRPM(double rpm){

      this.mRPM = rpm;
      addRequirements(FlywheelSubsystem.this);

    }

    @Override
    public void initialize() {
      mFlywheelLoop.reset(VecBuilder.fill(getRadiansPerSecond()));
    }

    @Override
    public void execute() {
        mFlywheelLoop.setNextR(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(mRPM)));
        mFlywheelLoop.correct(VecBuilder.fill(getRadiansPerSecond()));
        mFlywheelLoop.predict(0.020);
        double mNextVolts = mFlywheelLoop.getU(0);
        mProtagonist.setVoltage(mNextVolts);
    }

    @Override
    public void end(boolean interrupted) {
        mFlywheelLoop.setNextR(VecBuilder.fill(0.0));
    }

  }
}
