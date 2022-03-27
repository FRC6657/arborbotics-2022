package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.custom.ArborMath;
import frc.robot.subsystems.VisionSubsystem.VisionSupplier;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class HoodSubsystem extends SubsystemBase implements Loggable{

  private final CANSparkMax mMotor = new CANSparkMax(Constants.CAN.kHood, MotorType.kBrushless);

  @Config.PIDController(tabName = "Shooter")
  private PIDController mPID = new PIDController(0.44, 0, 0);

  public double mTargetAngle, mCurrentAngle, mPIDEffort, mFFEffort;

  @Config.Command(tabName = "Shooter", name = "Reset Hood Angle")
  public Command resetAngle = new resetAngle();

  VisionSupplier vision;

  public HoodSubsystem(VisionSupplier vision) {
    configureMotor();
    setTargetAngle(0);
    mPID.setTolerance(1, 5);
    this.vision = vision;
  }

  public void configureMotor(){
    mMotor.setSmartCurrentLimit(25);
    mMotor.restoreFactoryDefaults();
    mMotor.setIdleMode(IdleMode.kBrake);
    mMotor.getEncoder().setPosition(0);
    mMotor.getEncoder().setPositionConversionFactor(Constants.Hood.kAngleConversionFactor);
  }
  
  public void runHood(){
    
    mCurrentAngle = getAngle();

    // if(vision.hasTarget()){
    //   setTargetAngle(InterpolatingTable.get(vision.getDistance()).hoodAngle);
    // }

    mPIDEffort = mPID.calculate(mCurrentAngle, mTargetAngle);

    mMotor.setVoltage(mPIDEffort);

  }

  @Config(tabName = "Shooter", name = "Set Hood Angle")
  public void setTargetAngle(double newTarget){
    mTargetAngle = MathUtil.clamp(newTarget,0,50);
  }

  public void set(double percent){
    mMotor.set(percent);
  }

  @Log(tabName = "Shooter", name = "Hood Angle")
  public double getAngle(){
    return mMotor.getEncoder().getPosition();
  }

  @Log(tabName = "Shooter", name = "Hood Ready")
  public boolean ready(){
    return ArborMath.inTolerance(Math.abs(mTargetAngle - mCurrentAngle), 1) && mTargetAngle != 0;
  }

  public void stop(){
    setTargetAngle(0);
    mMotor.stopMotor();
  }

  private class resetAngle extends CommandBase{
    @Override
    public void initialize() {
      mMotor.getEncoder().setPosition(0);
    }
  }

  @Override
  public void periodic() {
    runHood();
    //mMotor.set(-0.1);
  }
}
