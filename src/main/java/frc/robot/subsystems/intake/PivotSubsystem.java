// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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
    pivotMotor.setNeutralMode(NeutralMode.Brake);
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  private void pivot(double percent){
    pivotMotor.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
