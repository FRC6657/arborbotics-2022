// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase {
  
  private DoubleSolenoid mLeftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid mRightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  public ExtensionSubsystem() {
    retract();
  }

  public void extend(){
    mLeftPiston.set(Value.kReverse);
    mRightPiston.set(Value.kReverse);
  }
  public void retract(){
    mLeftPiston.set(Value.kForward);
    mRightPiston.set(Value.kForward);
  }

}
