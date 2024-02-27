package frc.robot.subsystems.camera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.camera.PhotonVisionCamera.CameraState;

public class PhotonVisionCameraTelemetry {

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Photon Vision Camera");
    private final IntegerArrayPublisher aprilTagsSeen = table.getIntegerArrayTopic("Seeing AprilTags").publish();
    private final DoubleArrayPublisher pose = table.getDoubleArrayTopic("Robot Pose from Camera").publish();
    private final DoubleArrayPublisher robotToTarget = table.getDoubleArrayTopic("Distance from AprilTag").publish();

    public PhotonVisionCameraTelemetry() {

    }

    public void telemetrize(CameraState state) {
        aprilTagsSeen.set(state.aprilTagsSeen);
        var statePose = state.robotFieldPose.toPose2d();
        pose.set(new double[] {
                statePose.getX(),
                statePose.getY(),
                statePose.getRotation().getDegrees()
        });
        // roll x pitch y yaw z
        var transformPose = state.robotToTarget;
        robotToTarget.set(new double[] {
                transformPose.getX(),
                transformPose.getY(),
                transformPose.getZ(),
                Rotation2d.fromRadians(transformPose.getRotation().getX()).getDegrees(),
                Rotation2d.fromRadians(transformPose.getRotation().getY()).getDegrees(),
                Rotation2d.fromRadians(transformPose.getRotation().getZ()).getDegrees()

        });

    }
}
