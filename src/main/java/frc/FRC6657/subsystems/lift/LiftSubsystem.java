// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.lift;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;


public class LiftSubsystem extends SubsystemBase {

  private final WPI_TalonSRX leftMotor;
  private final WPI_TalonSRX rightMotor;
  private final DutyCycleEncoder leftEncoder;
  private final DutyCycleEncoder rightEncoder;  
  private double leftSetPoint, rightSetPoint;
  
  public LiftSubsystem() {
    rightMotor = new WPI_TalonSRX(Constants.kRightLiftID);
    leftMotor = new WPI_TalonSRX(Constants.kLeftLiftID);
    rightEncoder = new DutyCycleEncoder(0); 
    leftEncoder = new DutyCycleEncoder(1);
    
    leftEncoder.setDistancePerRotation(Math.PI);
    rightEncoder.setDistancePerRotation(Math.PI);

  }

  
  public double getError(double setpoint) {
    return setpoint - rightEncoder.getDistance();
  }

  public void setLeftSetPoint(double setpoint) {
    leftSetPoint = setpoint;
  }

  public void setRightSetPoint(double setpoint) {
    rightSetPoint = setpoint;
  }

  public void run() {

    leftMotor.set(Constants.Lift.kP * getError(leftSetPoint));
    rightMotor.set(Constants.Lift.kP * getError(rightSetPoint));

  }

}