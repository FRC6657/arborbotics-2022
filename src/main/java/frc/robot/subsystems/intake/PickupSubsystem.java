// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class PickupSubsystem extends SubsystemBase implements Loggable {

  private WPI_TalonSRX pickupMotor = new WPI_TalonSRX(Constants.kPickupID);

  public PickupSubsystem() {
    configMotor();
  }

  private void configMotor() {
    pickupMotor.configFactoryDefault();
    pickupMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Config(rowIndex = 0, columnIndex = 0, width = 2, height = 1, name = "Pickup Speed", defaultValueNumeric = Constants.kPickupSpeed)
  private void run(double percent) {
    pickupMotor.set(percent);
  }

  private void stop() {
    pickupMotor.set(0);
  }

  @Override
  public void periodic() {

  }
  public class PickupCommand extends CommandBase {

    public PickupCommand(PickupSubsystem pickup){
      addRequirements(PickupSubsystem.this);
    }

    @Override
    public void initialize() {
      run(Constants.kPickupSpeed);
    }
    @Override
    public void end(boolean interrupted) {
      stop();
    }
  }
}
