package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class PositionUtils {
    public static boolean isPointInTriangle(Translation2d pt, Translation2d v1, Translation2d v2, Translation2d v3) {
        boolean b1, b2, b3;

        b1 = sign(pt, v1, v2) < 0.0f;
        b2 = sign(pt, v2, v3) < 0.0f;
        b3 = sign(pt, v3, v1) < 0.0f;

        return ((b1 == b2) && (b2 == b3));
    }

    private static double sign(Translation2d p1, Translation2d p2, Translation2d p3) {
        return (p1.getX() - p3.getX()) * (p2.getY() - p3.getY()) - (p2.getX() - p3.getX()) * (p1.getY() - p3.getY());
    }
}
