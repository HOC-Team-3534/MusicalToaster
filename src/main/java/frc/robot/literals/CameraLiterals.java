package frc.robot.literals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraLiterals {

    private final double distanceInFrontOfCenterOfRobot_x;
    private final double distanceOffsetSideToSide_y;
    private final double heightOfCameraOffGround_z;
    private final Rotation2d tiltOfCamera_pitch;

    private final String cameraName;

    public CameraLiterals(double distanceInFrontOfCenterOfRobot_x, double distanceOffsetSideToSide_y,
            double heightOfCameraOffGround_z, Rotation2d tiltOfCamera_pitch) {
        this(distanceInFrontOfCenterOfRobot_x, distanceOffsetSideToSide_y, heightOfCameraOffGround_z,
                tiltOfCamera_pitch, "3534camera");
    }

    public CameraLiterals(double distanceInFrontOfCenterOfRobot_x, double distanceOffsetSideToSide_y,
            double heightOfCameraOffGround_z, Rotation2d tiltOfCamera_pitch, String name) {
        this.distanceInFrontOfCenterOfRobot_x = distanceInFrontOfCenterOfRobot_x;
        this.distanceOffsetSideToSide_y = distanceOffsetSideToSide_y;
        this.heightOfCameraOffGround_z = heightOfCameraOffGround_z;
        this.tiltOfCamera_pitch = tiltOfCamera_pitch;
        this.cameraName = name;
    }

    public Transform3d getRobotToCameraTransform() {
        return new Transform3d(
                new Translation3d(distanceInFrontOfCenterOfRobot_x, distanceOffsetSideToSide_y,
                        heightOfCameraOffGround_z),
                new Rotation3d(0, tiltOfCamera_pitch.getRadians(), 0));
    }

    public String getName() {
        return cameraName;
    }

}
