package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  
    private static WPI_TalonSRX mLeftMotor = new WPI_TalonSRX(Constants.CAN.kLeftLift);
    private static WPI_TalonSRX mRightMotor = new WPI_TalonSRX(Constants.CAN.kRightLift);

  public LiftSubsystem() {
    configureMotor();
  }

  public void configureMotor(){
    mLeftMotor.configFactoryDefault();
    mLeftMotor.setNeutralMode(NeutralMode.Coast);
    mLeftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 80, 1));
    mLeftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 250);
    mLeftMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 250);
    mLeftMotor.stopMotor();

    mRightMotor.configFactoryDefault();
    mRightMotor.setNeutralMode(NeutralMode.Coast);
    mRightMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 80, 1));
    mRightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);
    mRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 250);
    mRightMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 250);
    mRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 250);
    mRightMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 250);
    mRightMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 250);
    mRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 250);
    mRightMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 250);
    mRightMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 250);
    mRightMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 250);
    mRightMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 250);
    mRightMotor.stopMotor();

  }

  public void set(double percent){
    mRightMotor.set(percent);
    mLeftMotor.set(percent);
  }

  public void run(){
    set(Constants.Intake.kSpeed);
  }

  public void stop(){
    mLeftMotor.stopMotor();
    mRightMotor.stopMotor();
  }

}

