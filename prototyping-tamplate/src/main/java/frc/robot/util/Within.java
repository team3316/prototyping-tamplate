package frc.robot.util;

public class Within {
    public static boolean range(double wanted, double real, double tolerance) {
        return Math.abs(wanted - real) <= tolerance;
    }
}
