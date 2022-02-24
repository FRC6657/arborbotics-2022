// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.lift;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;


public class LiftSubsystem extends SubsystemBase {

  private final WPI_TalonSRX mLeftMotor;
  private final WPI_TalonSRX mRightMotor;
  
  public LiftSubsystem() {
    configureMotor();
    mRightMotor = new WPI_TalonSRX(Constants.kRightLiftID);
    mLeftMotor = new WPI_TalonSRX(Constants.kLeftLiftID);

  }

  public void setLeft(double percent) {
    mLeftMotor.set(percent);
  }
  public void setRight(double percent) {
    mRightMotor.set(percent);
  }
  public void set(double percent) {
    setLeft(percent);
    setRight(percent);
  }

  public void configureMotor() {  
    mLeftMotor.setNeutralMode(NeutralMode.Brake);
    mRightMotor.setNeutralMode(NeutralMode.Brake);
    mLeftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 80, 85, 0.1));
    mRightMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 80, 85, 0.1));

  }

}