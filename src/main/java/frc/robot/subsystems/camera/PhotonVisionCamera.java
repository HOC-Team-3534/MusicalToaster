package frc.robot.subsystems.camera;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;

public class PhotonVisionCamera extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera(Constants.ROBOT.getCameraLiterals().getName());
    AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonVisionCameraTelemetry photonVisionCameraTelemetry = new PhotonVisionCameraTelemetry();

    final PhotonPoseEstimator photonPoseEstimator;

    private static final boolean enabled = false;
    private static PhotonVisionCamera INSTANCE;

    public static Optional<PhotonVisionCamera> createInstance() {
        if (INSTANCE != null) {
            return Optional.of(INSTANCE);
        }
        if (!enabled)
            return Optional.empty();
        INSTANCE = new PhotonVisionCamera();
        return Optional.of(INSTANCE);
    }

    public static Optional<PhotonVisionCamera> getInstance() {
        return Optional.ofNullable(INSTANCE);
    }

    public PhotonVisionCamera() {
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                Constants.ROBOT.getCameraLiterals().getRobotToCameraTransform());
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        RobotContainer.getPose().ifPresent((referencePose) -> {
            photonPoseEstimator.setReferencePose(referencePose);
            var estimatedPose = photonPoseEstimator.update();

            if (estimatedPose.isPresent()) {
                var estimate = estimatedPose.get();
                var targetId = estimate.targetsUsed.get(0).getFiducialId();

                var targetPose = aprilTagFieldLayout.getTagPose(targetId).get();

                CommandSwerveDrivetrain.getInstance().ifPresent((drivetrain) -> drivetrain
                        .addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds));

                m_cachedState.robotToTarget = targetPose.minus(estimate.estimatedPose);
                for (int i = 0; i < estimate.targetsUsed.size() && i < 2; i++) {
                    m_cachedState.aprilTagsSeen[i] = estimate.targetsUsed.get(i).getFiducialId();
                }
                photonVisionCameraTelemetry.telemetrize(m_cachedState);
            }
        });
    }

    public class CameraState {
        public Transform3d robotToTarget;
        public Pose3d robotFieldPose;
        public long[] aprilTagsSeen = new long[2];
    }

    final CameraState m_cachedState = new CameraState();
}
