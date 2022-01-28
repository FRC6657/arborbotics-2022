package frc.FRC6657.subsystems.blinkin;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;

public class BlinkinSubsystem extends SubsystemBase {
  
private Spark blinkinController;

  public BlinkinSubsystem() {
    blinkinController = new Spark(Constants.kBlinkinID);

  }

  @Override
  public void periodic() {
    

  }

  public void intakeColor() {
    blinkinController.set(-0.85); // shot red

  }

  public void defaultColor() {
    blinkinController.set(0.75); // green 

  }


public class runDefault extends CommandBase { 

  public void run() {
  defaultColor();

   }
 }

public class runIntake extends CommandBase {

  public void run() {
    intakeColor();

  }
}

}

