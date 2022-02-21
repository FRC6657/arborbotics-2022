package frc.FRC6657.subsystems.blinkin;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.rev.Blinkin;
import frc.FRC6657.custom.rev.Blinkin.BlinkinLEDPattern;

public class BlinkinSubsystem extends SubsystemBase {
  
private Blinkin mBlinkin;

  public BlinkinSubsystem() {
    mBlinkin = new Blinkin(Constants.kBlinkinID);
    setIdleColor();
  }

  public void setIntakingColor() {
    setBlinkinColor(Constants.BlinkinColors.kIdle);
  }

  public void setIdleColor(){
    setBlinkinColor(Constants.BlinkinColors.kIntake);
  }

  public void setReadyFlywheelColor() {
    setBlinkinColor(Constants.BlinkinColors.kReadyFlywheel);
  }
    
  public void setNotReadyFlywheelColor() {
    setBlinkinColor(Constants.BlinkinColors.kNotReadyFlywheel);
  }

  private void setBlinkinColor(BlinkinLEDPattern pattern){
    mBlinkin.setLEDMode(pattern);
  }
}

