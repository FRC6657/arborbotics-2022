// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakePistonsSubsystem extends SubsystemBase implements Loggable{

  private DoubleSolenoid mLeftPiston = new DoubleSolenoid(7, PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid mRightPiston = new DoubleSolenoid(7, PneumaticsModuleType.CTREPCM, 6, 7);

  public IntakePistonsSubsystem(){
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

  @Log(tabName = "Intake")
  public boolean extended(){
    return mLeftPiston.get() != Value.kForward;
  }

}
