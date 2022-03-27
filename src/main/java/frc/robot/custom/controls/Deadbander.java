package frc.robot.custom.controls;

public class Deadbander {
    
    /**
     * 
     * Author: Andrew Card
     * 
     * @param input  Value to be corrected
     * @param thresh Deadband threshold
     * @return Deadbanded value
     */
    public static double applyLinearScaledDeadband(double input, double thresh) {
        if (Math.abs(input) < thresh) {
            return 0;
        } else {
            return (input - (Math.abs(input) / input) * thresh) / (1.0 - thresh);
        }
    }

    /**
     * @param input Value to be correctde
     * @param thresh Deadband threshold
     * @return Deadbanded Value
     */
    public double applySimpleDeadband(double input, double thresh){
        if(input > thresh){
            return input;
        }
        return 0;
    }

}
