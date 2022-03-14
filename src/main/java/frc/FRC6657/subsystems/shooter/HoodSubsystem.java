// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.FRC6657.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FRC6657.Constants;
import frc.FRC6657.custom.ArborMath;
import frc.FRC6657.subsystems.shooter.interpolation.InterpolatingTable;
import frc.FRC6657.subsystems.vision.VisionSubsystem.VisionSupplier;

@SuppressWarnings("unused")
public class HoodSubsystem extends SubsystemBase {

    private CANSparkMax mMotor;
    private WPI_TalonSRX mSimMotor;

    private Mechanism2d hoodMech = new Mechanism2d(5, 5);
    private MechanismRoot2d hoodRoot = hoodMech.getRoot("Hood", 4, 1.5);
    private MechanismLigament2d hoodAngle = hoodRoot.append(new MechanismLigament2d("Hood-Angle", 3, -180, 4, new Color8Bit(247,241,69)));
    private MechanismLigament2d hoodBase = hoodRoot.append(new MechanismLigament2d("Hood-Base", 3, -180, 2, new Color8Bit(52,224,153)));

    private PIDController mPidController = Constants.Hood.kPIDController;

    double angleSetpoint = 0;
    boolean homing = false;

    private Boolean visionEnabled = true;

    private DoubleSupplier estDistance;
    private VisionSupplier mVision;

    public HoodSubsystem(VisionSupplier vision, DoubleSupplier estDistance) {
        mMotor = new CANSparkMax(Constants.kHoodID, MotorType.kBrushless);
        configureMotor();

        this.mVision = vision;
        this.estDistance = estDistance;

        SmartDashboard.putData("Hood", hoodMech);

    }

    private void configureMotor() {
        mMotor.setSmartCurrentLimit(30);
        mMotor.setIdleMode(IdleMode.kBrake);
        mMotor.getEncoder().setPositionConversionFactor(Constants.Hood.kRatio/360); //Converts Encoder Reading to Degrees
    }

    public void set(double percent) {
        mMotor.set(percent);
    }

    public void stop() {
        set(0);
    }

    public void setAngle(double angle){
        angleSetpoint = MathUtil.clamp(angle, 0, 45);
    }
    
    public double getAngle(){
        if(RobotBase.isReal()){
            return mMotor.getEncoder().getPosition();
        }else{
            return angleSetpoint;
        }
    }

    public boolean atTarget(){
        return Math.abs(angleSetpoint - getAngle()) < 1;
    }

    @Override
    public void periodic() {
        if (homing == false){
            if(visionEnabled){
                if(mVision.hasTarget()){
                    double target = InterpolatingTable.get(mVision.getDistance()).hoodAngle;
                    hoodAngle.setAngle(-180-target);
                }else{
                    double target = InterpolatingTable.get(estDistance.getAsDouble()).hoodAngle;
                    hoodAngle.setAngle(-180-target);
                }
            }else{
                hoodAngle.setAngle(-180-angleSetpoint); //Visualizer
                mMotor.setVoltage(mPidController.calculate(getAngle(), angleSetpoint));
            }
        }
    }

    public class Home extends CommandBase{

        private final Timer timer = new Timer();

        @Override
        public void initialize() {

            homing = true;

            if(RobotBase.isSimulation()){
                setAngle(0);
            }

            timer.reset();
            timer.start();

        }

        @Override
        public void end(boolean interrupted) {
            timer.stop();
            stop();
            mMotor.getEncoder().setPosition(0);
            homing = false;
        }

        @Override
        public boolean isFinished() {
            if(RobotBase.isSimulation()){
                return true;
            }else{
                return (timer.get() > 0.5) && mMotor.getOutputCurrent() > 10; //TODO TUNE
            }
        }

    }


}
