// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.custom.rev;

import frc.FRC6657.custom.rev.Blinkin.BlinkinLEDPattern;

public class BlinkinIndicator {
    public final String name;
    public final int priority;
    public final BlinkinLEDPattern pattern;
    public BlinkinIndicator(String name, int priority, BlinkinLEDPattern pattern){
        this.name = name;
        this.priority = priority;
        this.pattern = pattern;
    }
}
