// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.lift;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;


public class LiftSubsystem extends SubsystemBase {

  private final WPI_TalonSRX mLeftMotor;
  private final WPI_TalonSRX mRightMotor;
  
  public LiftSubsystem() {
    mRightMotor = new WPI_TalonSRX(Constants.kRightLiftID);
    mLeftMotor = new WPI_TalonSRX(Constants.kLeftLiftID);

    mLeftMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 250);
    mRightMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 250);

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

}