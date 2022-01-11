// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotSubsystem extends SubsystemBase {
  
  private WPI_TalonSRX pivotMotor = new WPI_TalonSRX(Constants.kPivotID);

  public PivotSubsystem() {}

  private void configureMotor() {
    pivotMotor.configFactoryDefault();
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    pivotMotor.configNeutralDeadband(0.001);
    //Set Output Behaviors
    pivotMotor.configNominalOutputForward(0);
    pivotMotor.configNominalOutputReverse(0);
    pivotMotor.configPeakOutputForward(1);
    pivotMotor.configPeakOutputReverse(-1);
    //Motion Magic Config
    pivotMotor.selectProfileSlot(Constants.pivotSlotIdx, Constants.pivotSlotIdx);
    pivotMotor.config_kF(Constants.pivotSlotIdx, Constants.pivotKF);
    pivotMotor.config_kP(Constants.pivotSlotIdx, Constants.pivotKP);
    pivotMotor.config_kI(Constants.pivotSlotIdx, Constants.pivotKI);
    pivotMotor.config_kD(Constants.pivotSlotIdx, Constants.pivotKD);
    //Velocity Cruise
    pivotMotor.configMotionCruiseVelocity(300);
    pivotMotor.setSelectedSensorPosition(0, Constants.pivotSlotIdx, 0);
  }

  private void pivot(double percent){
    pivotMotor.set(percent);
  }

  private void extend(){
    pivotMotor.set(ControlMode.MotionMagic, Constants.kExtendedPos);
  }

  private void retract(){
    pivotMotor.set(ControlMode.MotionMagic, Constants.kRetractedPos);
  }

  @Override
  public void periodic() {
    double motorOutput = pivotMotor.getMotorOutputPercent();
  }
}
