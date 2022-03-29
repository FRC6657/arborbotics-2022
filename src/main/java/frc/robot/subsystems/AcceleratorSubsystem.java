package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AcceleratorSubsystem extends SubsystemBase {
  
  private static WPI_TalonSRX mMotor = new WPI_TalonSRX(Constants.CAN.kAccelerator);

  public AcceleratorSubsystem() {
    configureMotor();
  }

  public void configureMotor(){
    mMotor.configFactoryDefault();
    mMotor.setNeutralMode(NeutralMode.Coast);
    mMotor.setInverted(true);
    mMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));
    mMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 250);
    mMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 250);
    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 250);
    mMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 250);
    mMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 250);
    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 250);
    mMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 250);
    mMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 250);
    mMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 250);
    mMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 250);
    mMotor.stopMotor();
  }

  public void set(double percent){
    mMotor.set(percent);
  }

  public void start(){
    set(Constants.Accelerator.kSpeed);
  }

  public void reverse(){
    set(-Constants.Accelerator.kSpeed);
  }

  public void stop(){
    mMotor.stopMotor();
  }

}

