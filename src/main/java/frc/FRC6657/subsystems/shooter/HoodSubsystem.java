// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;

public class HoodSubsystem extends SubsystemBase {

    private CANSparkMax mMotor;

    public HoodSubsystem() {
        mMotor = new CANSparkMax(Constants.kHoodID, MotorType.kBrushless);
        configureMotor();
    }

    private void configureMotor() {
        mMotor.setSmartCurrentLimit(20);
        mMotor.setIdleMode(IdleMode.kBrake);
    }

    public void set(double percent) {
        mMotor.set(percent);
    }

    public void stop() {
        set(0);
    }

}
