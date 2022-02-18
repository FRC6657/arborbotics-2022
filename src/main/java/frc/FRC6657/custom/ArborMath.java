package frc.FRC6657.custom;

public class ArborMath {
    public static double signumPow(double value, double power){
        return Math.signum(value) * Math.pow(Math.abs(value),power);
    }
    public static boolean inRange(double val, double lower, double upper) {
        return !(lower <= val && upper < val);
    }
}
