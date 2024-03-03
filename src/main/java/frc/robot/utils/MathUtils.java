package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class MathUtils {
    public static double applyUpperAndLowerLimit(double value, double lower, double upper) {
        if (value < lower)
            value = lower;
        if (value > upper)
            value = upper;
        return value;
    }

    public static boolean withinTolerance(Rotation2d value, Rotation2d tolerance) {
        return withinTolerance(value.getDegrees(), tolerance.getDegrees());
    }

    public static boolean withinTolerance(double value, double tolerance) {
        return Math.abs(value) <= Math.abs(tolerance);
    }

}
