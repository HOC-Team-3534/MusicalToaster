package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SmartDashboardUtils {

    static final Translation2d FIELD2D_OFFSET = new Translation2d(1.0525, 0.434);
    static final double FIELD2D_SCALE = 0.96;

    public static Pose2d getPose2dForField2d(Pose2d pose) {
        return new Pose2d(pose.getTranslation().times(FIELD2D_SCALE).plus(FIELD2D_OFFSET), pose.getRotation());
    }

}
