// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class ExtensionSubsystem extends SubsystemBase implements Loggable {

  private final DoubleSolenoid mPiston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private final DoubleSolenoid mPiston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  public ExtensionSubsystem() {
    setDefaultState();
  }

  @Override
  public void periodic() {

  }

  private final void setDefaultState() {
    mPiston1.set(Value.kReverse);
    mPiston2.set(Value.kReverse);
  }

  public final void toggleState() {
    mPiston1.toggle();
    mPiston2.toggle();
  }
  
}