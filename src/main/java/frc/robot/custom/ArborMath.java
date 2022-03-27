package frc.robot.custom;

public class ArborMath {
    /**
     * @param val Input Value
     * @param power Power to be raised to
     * @return Output Value
     * 
     * Raises a value to a power while maintaining the sign of the original value
     * 
     */
    public static double signumPow(double val, double power){
        return Math.signum(val) * Math.pow(Math.abs(val),power);
    }

    public static double normalizeAngle(double inHeading) {
        if(inHeading > 360){
          inHeading -= 360;
          return normalizeAngle(inHeading);
        } else if (inHeading < 0){
          inHeading += 360;
          return normalizeAngle(inHeading);
        }
        else return inHeading;
      }

    /**
     * @param val Input value
     * @param lower Low end of the range
     * @param upper High end of the range
     * @return If the input value is within the range
     */
    public static boolean inRange(double val, double lower, double upper) {
        return !(lower <= val && upper < val);
    }

    /**
     * @param val Input value
     * @param tolerance tollerance around a 0 point
     * @return If the input is in the tollerance
     */
    public static boolean inTolerance(double val, double tolerance){
        return inRange(val, -tolerance, tolerance);
    }
}
