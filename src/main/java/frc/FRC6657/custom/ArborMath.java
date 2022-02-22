package frc.FRC6657.custom;

public class ArborMath {
    public static double signumPow(double val, double power){
        return Math.signum(val) * Math.pow(Math.abs(val),power);
    }
    public static boolean inRange(double val, double lower, double upper) {
        return !(lower <= val && upper < val);
    }
    public static boolean inTolerance(double val, double tollerance){
        return inRange(val, -tollerance, tollerance);
    }
}
