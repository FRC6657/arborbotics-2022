package frc.FRC6657.subsystems.blinkin;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.rev.Blinkin;
import frc.FRC6657.custom.rev.BlinkinIndicator;
import frc.FRC6657.custom.rev.Blinkin.BlinkinLEDPattern;

public class BlinkinSubsystem extends SubsystemBase {
  
  //Creates a new blinkin
  private Blinkin mBlinkin;

  public BlinkinSubsystem() {
    mBlinkin = new Blinkin(Constants.kBlinkinID); //Assigns the blinkin to its PWM ID
    setBlinkinColor(Constants.BlinkinColors.kIdle); //Sets the Idle color
  }

  /**
   * @param pattern Color to set the LED's to
   */
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

