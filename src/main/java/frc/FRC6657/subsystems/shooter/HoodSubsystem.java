// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.controls.Deadbander;

public class HoodSubsystem extends SubsystemBase {

    private CANSparkMax mMotor;

    public HoodSubsystem() {
        mMotor = new CANSparkMax(Constants.kHoodID, MotorType.kBrushless);
        configMotor();
    }

    private void configMotor() {
        mMotor.setSmartCurrentLimit(20);
        mMotor.setIdleMode(IdleMode.kBrake);
    }

    private void set(double percent) {
        mMotor.set(percent);
    }

    private void stop() {
        set(0);
    }
    
    public void run(double speed){
        set(Deadbander.applyLinearScaledDeadband(speed, 0.1));
    }
}
