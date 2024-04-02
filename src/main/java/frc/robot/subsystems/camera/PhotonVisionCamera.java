package frc.robot.subsystems.camera;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.utils.MathUtils;

public class PhotonVisionCamera extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera(Constants.ROBOT.getCameraLiterals().getName());
    AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonVisionCameraTelemetry photonVisionCameraTelemetry = new PhotonVisionCameraTelemetry();

    final PhotonPoseEstimator photonPoseEstimator;

    private static final boolean enabled = true;
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
        RobotState.getPose().ifPresent((referencePose) -> {
            photonPoseEstimator.setReferencePose(referencePose);
            var estimatedPose = photonPoseEstimator.update();

            if (estimatedPose.isPresent()) {
                var estimate = estimatedPose.get();

                var targetId = estimate.targetsUsed.get(0).getFiducialId();
                var targetPose = aprilTagFieldLayout.getTagPose(targetId).get();

                var ambiguityOutOfRange = estimate.targetsUsed.get(0).getPoseAmbiguity() > 0.2;

                m_cachedState.robotToTarget = targetPose.minus(estimate.estimatedPose);

                var pitch = Rotation2d.fromRadians(m_cachedState.robotToTarget.getRotation().getY());

                if (!ambiguityOutOfRange && MathUtils.withinTolerance(pitch, Rotation2d.fromDegrees(5))) {
                    if (Constants.EnabledDebugModes.updatePoseWithVisionEnabled) {
                        CommandSwerveDrivetrain.getInstance().ifPresent((drivetrain) -> {
                            drivetrain
                                    .addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
                            m_cachedState.lastTimeUpdated.restart();
                            m_cachedState.updatedRobotPose = drivetrain.getState().Pose.times(1);
                        });
                    }

                    for (int i = 0; i < estimate.targetsUsed.size() && i < 2; i++) {
                        m_cachedState.aprilTagsSeen[i] = estimate.targetsUsed.get(i).getFiducialId();
                    }
                }

                photonVisionCameraTelemetry.telemetrize(m_cachedState);
            }
        });
    }

    public CameraState getState() {
        return this.m_cachedState;
    }

    public class CameraState {

        CameraState() {
            lastTimeUpdated.restart();
        }

        public Transform3d robotToTarget;
        public Pose3d robotFieldPose;
        public long[] aprilTagsSeen = new long[2];
        public Timer lastTimeUpdated = new Timer();
        public Pose2d updatedRobotPose;
    }

    final CameraState m_cachedState = new CameraState();
}
