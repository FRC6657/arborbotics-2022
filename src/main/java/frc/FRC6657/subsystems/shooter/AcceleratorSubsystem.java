// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import io.github.oblarg.oblog.Loggable;

public class AcceleratorSubsystem extends SubsystemBase implements Loggable{
  
  private final WPI_TalonSRX mMotor;

  public AcceleratorSubsystem() {
    mMotor = new WPI_TalonSRX(Constants.kAcceleratorID);
    configureMotor();
  }

  private void configureMotor(){
    mMotor.configFactoryDefault();
    mMotor.setNeutralMode(NeutralMode.Coast);
    mMotor.setInverted(false);
    mMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0.5));
  }

  private void set(double percent){
    mMotor.set(percent);
  }


  public void run(){
    set(Constants.Accelerator.kSpeed);
  }

}
