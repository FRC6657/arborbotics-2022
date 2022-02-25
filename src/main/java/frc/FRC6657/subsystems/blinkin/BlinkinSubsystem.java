package frc.FRC6657.subsystems.blinkin;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.rev.Blinkin;
import frc.FRC6657.custom.rev.Blinkin.BlinkinLEDPattern;

public class BlinkinSubsystem extends SubsystemBase {
  
  private Blinkin mBlinkin;
  private int currentPriority = -1;

  public BlinkinSubsystem() {
    mBlinkin = new Blinkin(Constants.kBlinkinID);
    setBlinkinColor(Constants.BlinkinColors.kIdle);
  }

  public void setBlinkinColor(BlinkinLEDPattern pattern){
    mBlinkin.setLEDMode(pattern);
    currentPriority = -1;
  }

  public void setBlinkinColor(BlinkinLEDPattern pattern, int priority){
    if(priority > currentPriority){
      setBlinkinColor(pattern);
      currentPriority = priority;
    }
  }
}

