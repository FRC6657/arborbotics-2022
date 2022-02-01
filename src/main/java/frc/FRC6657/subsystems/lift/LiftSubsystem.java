// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.lift;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;

public class LiftSubsystem extends SubsystemBase {

  private final WPI_TalonSRX mMotor;
  private final Encoder encoder; 
  private final double encoderCPR = 8192;
  private final double kP = 1;

  
  public LiftSubsystem() {
    mMotor = new WPI_TalonSRX(Constants.kLiftID);
    encoder = new Encoder(0,1); 

    double distancePerPulse = Math.PI / encoderCPR;

    encoder.setDistancePerPulse(distancePerPulse);

  }

  
  public double calculateError(double setpoint) {
    double error; 

    error = setpoint - encoder.getDistancePerPulse();

    return error;

  }

  public double getX(double error) {
    double x;

    x = Math.PI / encoderCPR; 

    return x;

  }


  public void run() {

    double setpoint = encoder.getDistance();
    encoder.setDistancePerPulse(getX(calculateError(setpoint)));
    
    mMotor.set(kP * calculateError(setpoint));

  }

  @Override
  public void periodic() {
    

  }
}