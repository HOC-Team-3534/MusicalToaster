package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretUtils {

    public static Rotation2d calculateTargetAzimuthWithinLimits(Rotation2d target, Rotation2d current,
            double lowerLimitDegrees,
            double upperLimitDegrees) {
        double a = current.getDegrees();
        double b = target.getDegrees();
        a %= 360;
        a += a < 0 ? 360 : 0;
        b %= 360;
        b += b < 0 ? 360 : 0;
        double difference = b - a;
        double shiftInCurrentAngle = Math.abs(difference) < 180 ? difference
                : difference < 0 ? difference + 360 : difference - 360;

        double outputTarget = current.getDegrees() + shiftInCurrentAngle;
        while (outputTarget > upperLimitDegrees) {
            outputTarget -= 360;
        }
        while (outputTarget < lowerLimitDegrees) {
            outputTarget += 360;
        }
        return Rotation2d.fromDegrees(outputTarget);

    }

}
