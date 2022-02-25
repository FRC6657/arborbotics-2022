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
    if(pattern.toString() == BlinkinLEDPattern.COLOR_WAVES_FOREST_PALETTE.toString()){
      System.out.println("Blink Set to Idle");
    }
    currentPriority = -1;
  }

  public void setBlinkinColor(BlinkinLEDPattern pattern, int priority){
    System.out.println("Current Color:" + mBlinkin.getLEDMode() + "|||||Incoming Color: " + pattern.toString());
    if(priority > currentPriority){
      setBlinkinColor(pattern);
      System.out.println("Output: " + mBlinkin.getLEDMode());
      currentPriority = priority;
    }
  }
}

