// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class PickupSubsystem extends SubsystemBase implements Loggable {

  private WPI_TalonSRX mMotor;

  public PickupSubsystem() {
    mMotor = new WPI_TalonSRX(Constants.kPickupID);
    configMotor();
  }

  private void configMotor() {
    mMotor.setInverted(true);
    mMotor.setNeutralMode(NeutralMode.Coast);

  }
  
  @Config(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "Pickup Speed", defaultValueNumeric = 0) //Allows for easy intake testing
  private void set(double percent) {
    mMotor.set(percent);
  }

  public void run(){
    set(Constants.Intake.kSpeed);
  }

  public void stop() {
    set(0);
  }
}

