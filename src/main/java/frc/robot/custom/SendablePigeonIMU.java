package frc.robot.custom;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Fixes a bug with CTRE's Sendable Code that makes the Gyro Widget not function
 */
public class SendablePigeonIMU extends WPI_PigeonIMU{

    public SendablePigeonIMU(int deviceNumber) {
        super(deviceNumber);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getFusedHeading, null);
    }

}