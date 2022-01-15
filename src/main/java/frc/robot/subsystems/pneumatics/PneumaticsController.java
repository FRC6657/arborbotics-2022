// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;

public class PneumaticsController extends SubsystemBase implements Loggable {

  private Compressor compressor = new Compressor(Constants.kPCMID, PneumaticsModuleType.CTREPCM);

  public PneumaticsController() {
    configPneumatics();
  }

  private void configPneumatics() {
  }

  @Override
  public void periodic() {

  }
}
