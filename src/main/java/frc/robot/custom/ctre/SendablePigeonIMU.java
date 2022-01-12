package frc.robot.custom.ctre;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Fixes a bug with CTRE's Sendable Code that makes the Gyro Widget not function
 * Github Issue: https://github.com/CrossTheRoadElec/Phoenix-Releases/issues/23
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