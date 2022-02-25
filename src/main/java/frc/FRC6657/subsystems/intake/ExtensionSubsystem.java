// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ExtensionSubsystem extends SubsystemBase {
  
  private DoubleSolenoid mLeftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid mRightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  public ExtensionSubsystem() {
    retract();
  }

  @Log.BooleanBox(rowIndex = 0, columnIndex = 2, width = 1, height = 1, name = "Intake Extended", tabName = "Intake")
  public boolean getPistonState(){
    return mLeftPiston.get() == Value.kReverse;
  }

  @Config.ToggleSwitch(rowIndex = 1, columnIndex = 0, width = 1, height = 1, name = "Extension", tabName = "Intake")
  private void oblogControl(boolean extended){
    if(extended){
      extend();
    }else{
      retract();
    }
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
