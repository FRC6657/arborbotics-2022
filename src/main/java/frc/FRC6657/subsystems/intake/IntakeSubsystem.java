// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase implements Loggable {

  private WPI_TalonSRX mMotor = new WPI_TalonSRX(Constants.kPickupID);;

  private Timer mTimer = new Timer();

  public IntakeSubsystem() {
    configureMotor();
  }

  private void configureMotor() {
    mMotor.setInverted(true);
    mMotor.setNeutralMode(NeutralMode.Coast);
    mMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.1));

  }

  
  //@Config(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "Intake Speed", defaultValueNumeric = 0, tabName = "Intake") //Allows for easy intake testing
  public void set(double percent) {
    mMotor.set(percent);
    mTimer.start();
  }

  public void start(){
    set(Constants.Intake.kSpeed);
  }

  @Log(rowIndex = 1, columnIndex = 0, width = 2, height = 1, name = "Intake Speed", tabName = "IntakeSubsystem")
  public double getSpeed(){
    return mMotor.get();
  }

  public void stop() {
    set(0);
    mTimer.reset();
    mTimer.stop();
  }

  @Log.BooleanBox(rowIndex = 0, columnIndex = 1, width = 1, height = 1, name = "Ball Detected", tabName = "IntakeSubsystem")
  public boolean ballDetected(){
    return mTimer.get() < Constants.Intake.kStartupTime && mMotor.getStatorCurrent() > Constants.Intake.kBallCurrent;
  }

  public boolean active(){
    return mMotor.get() != 0;
  }

}

