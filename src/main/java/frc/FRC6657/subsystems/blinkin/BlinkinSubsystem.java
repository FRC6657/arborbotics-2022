package frc.FRC6657.subsystems.blinkin;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.rev.Blinkin;
import frc.FRC6657.custom.rev.BlinkinIndicator;
import frc.FRC6657.custom.rev.Blinkin.BlinkinLEDPattern;

public class BlinkinSubsystem extends SubsystemBase {
  
  private Blinkin mBlinkin;

  public BlinkinSubsystem() {
    mBlinkin = new Blinkin(Constants.kBlinkinID);
    setBlinkinColor(Constants.BlinkinColors.kIdle);
  }

  public void setBlinkinColor(BlinkinLEDPattern pattern){
    mBlinkin.setLEDMode(pattern);
  }

  public void setIndicator(BlinkinIndicator[] states){
    int i;
    int largest = 0;
    BlinkinLEDPattern selectedPattern = Constants.BlinkinColors.kIdle;
    for(i=0; i<states.length; i++){
      if (states[i].priority > largest){
        largest = states[i].priority;
        selectedPattern = states[i].pattern;
      }
    }

    System.out.println(selectedPattern);

  }
}

