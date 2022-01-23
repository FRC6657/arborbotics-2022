// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class PneumaticsController extends SubsystemBase implements Loggable {

  private final Compressor compressor = new Compressor(Constants.kPCMID, PneumaticsModuleType.CTREPCM);

  @Config
  public void runCompressor() {
    compressor.enableDigital();
  }

  @Config
  public void disableCompressor() {
    compressor.disable();
  }

  public boolean enabled() {
    return compressor.enabled();
  }

  public boolean pressureSwitch() {
    return compressor.getPressureSwitchValue();
  }

}
