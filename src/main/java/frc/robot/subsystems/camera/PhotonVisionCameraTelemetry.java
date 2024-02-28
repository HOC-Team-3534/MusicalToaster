package frc.robot.subsystems.camera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.camera.PhotonVisionCamera.CameraState;

public class PhotonVisionCameraTelemetry {

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Photon Vision Camera");
    private final IntegerArrayPublisher aprilTagsSeen = table.getIntegerArrayTopic("Seeing AprilTags").publish();
    private final DoubleArrayPublisher pose = table.getDoubleArrayTopic("Robot Pose from Camera").publish();
    private final DoublePublisher robotToTargetX = table.getDoubleTopic("Robot To Target X").publish();

    private final DoublePublisher robotToTargetY = table.getDoubleTopic("Robot To Target Y").publish();

    private final DoublePublisher robotToTargetZ = table.getDoubleTopic("Robot To Target Z").publish();

    private final DoublePublisher robotToTargetRoll = table.getDoubleTopic("Robot To Target Roll").publish();

    private final DoublePublisher robotToTargetPitch = table.getDoubleTopic("Robot To Target Pitch").publish();

    private final DoublePublisher robotToTargetYaw = table.getDoubleTopic("Robot To Target Yaw").publish();

    public void telemetrize(CameraState state) {
        if (state.aprilTagsSeen != null)
            aprilTagsSeen.set(state.aprilTagsSeen);
        var statePose = state.robotFieldPose;
        if (statePose != null) {
            var statePose2 = statePose.toPose2d();
            pose.set(new double[] {
                    statePose2.getX(),
                    statePose2.getY(),
                    statePose2.getRotation().getDegrees()
            });
        }
        // roll x pitch y yaw z

        var transformPose = state.robotToTarget;
        if (transformPose != null) {

            robotToTargetX.set(transformPose.getX());
            robotToTargetY.set(transformPose.getY());
            robotToTargetZ.set(transformPose.getZ());
            robotToTargetRoll.set(Rotation2d.fromRadians(transformPose.getRotation().getX()).getDegrees());
            robotToTargetPitch.set(Rotation2d.fromRadians(transformPose.getRotation().getY()).getDegrees());
            robotToTargetYaw.set(Rotation2d.fromRadians(transformPose.getRotation().getZ()).getDegrees());

        }

    }

}
