// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class PickupSubsystem extends SubsystemBase implements Loggable {

  private CANSparkMax pickupMotor = new CANSparkMax(Constants.kPickupID, MotorType.kBrushed);

  public PickupSubsystem() {
    configMotor();
  }

  private void configMotor() {
    
  }

  @Config(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "Pickup Speed", defaultValueNumeric = 0)
  private void set(double percent) {
    System.out.println(pickupMotor.getMotorType());
    pickupMotor.set(percent);
  }

  public void run(){
    set(Constants.kPickupSpeed);
  }

  public void stop() {
    set(0);
  }
}
