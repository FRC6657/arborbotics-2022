package frc.FRC6657.subsystems.blinkin;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.rev.Blinkin;
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
}

