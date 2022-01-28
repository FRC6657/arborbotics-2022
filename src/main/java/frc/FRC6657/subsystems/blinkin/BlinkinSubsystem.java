package frc.FRC6657.subsystems.blinkin;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    setBlinkinColor(BlinkinLEDPattern.SOLID_RED);

  }

  public void setIdleColor(){
    setBlinkinColor(BlinkinLEDPattern.SOLID_GREEN);
  }

  private void setBlinkinColor(BlinkinLEDPattern pattern){
    mBlinkin.setLEDMode(pattern);
  }

public class runDefault extends CommandBase { 
  public void run() {
    setIdleColor();
   }
 }

public class runIntake extends CommandBase {

  public void run() {
    setIntakingColor();
  }
}

}

