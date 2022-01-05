// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Robot extends TimedRobot {

  private WPI_TalonFX mFrontLeft = new WPI_TalonFX(1);
  private WPI_TalonFX mFrontRight = new WPI_TalonFX(2);
  private WPI_TalonFX mBackLeft = new WPI_TalonFX(3);
  private WPI_TalonFX mBackRight = new WPI_TalonFX(4);

  private DifferentialDrive mDrivetrain = new DifferentialDrive(mFrontLeft, mFrontRight);
  private double speed = 1;

  private XboxController mController = new XboxController(0);

  @Override
  public void robotInit() {
    configureMotors();
  }

  @Override
  public void teleopPeriodic() {
    mDrivetrain.curvatureDrive(
      deadband(-mController.getRawAxis(Axis.kLeftY.value), 0.1) * speed,
      deadband(mController.getRawAxis(Axis.kRightX.value), 0.1) * speed,
      mController.getRawButton(Button.kBumperRight.value)
    );
  }

  @Override
  public void disabledInit() {
    mDrivetrain.stopMotor();
  }

  private void configureMotors() {

    mFrontLeft.configFactoryDefault();
    mFrontRight.configFactoryDefault();
    mBackLeft.configFactoryDefault();
    mBackRight.configFactoryDefault();

    mBackLeft.follow(mFrontLeft);
    mBackRight.follow(mFrontRight);

    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);

    mFrontLeft.setInverted(TalonFXInvertType.Clockwise);
    mBackLeft.setInverted(TalonFXInvertType.Clockwise);
    mFrontRight.setInverted(TalonFXInvertType.CounterClockwise);
    mBackRight.setInverted(TalonFXInvertType.CounterClockwise);

    mFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    mFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    mFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5)); //| Enabled | 40a Limit | 45a Thresh | .5 sec Trigger Time
    mFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5)); //| Enabled | 40a Limit | 45a Thresh | .5 sec Trigger Time
    mBackLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5)); //| Enabled | 40a Limit | 45a Thresh | .5 sec Trigger Time
    mBackRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5)); //| Enabled | 40a Limit | 45a Thresh | .5 sec Trigger Time

  }

  /**
   * @param input Value to recieve a deadband
   * @param threshold The deadband threshold
   * 
   * This is a linear scaled deadband
   * 
   */
  public double deadband(double input, double threshold){
    if(Math.abs(input)<threshold){
      return 0;
    } else{
      return (input-(Math.abs(input)/input)*threshold ) / (1.0 - threshold);
    }
  }
}
