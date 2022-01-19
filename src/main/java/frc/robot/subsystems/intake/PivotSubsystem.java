// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import frc.robot.Constants;
import io.github.oblarg.oblog.annotations.Config;

public class PivotSubsystem extends SubsystemBase implements Loggable {

  private final DoubleSolenoid extender = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  public PivotSubsystem() {
    configExtender();
  }

  @Override
  public void periodic() {

  }

  private final void configExtender() {
    extender.set(Value.kReverse);
  }

  private final void toggle() {
    extender.toggle();
    System.out.println("We are toggling;");
  }

  public class ToggleCommand extends InstantCommand {

    @Override
    public final void initialize() {
      toggle();
    }
  }
}