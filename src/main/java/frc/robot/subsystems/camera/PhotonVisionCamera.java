package frc.robot.subsystems.camera;

import java.util.Optional;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;

public class PhotonVisionCamera extends SubsystemBase {

    final double UpdateFrequency = 50.0;

    ReadWriteLock m_stateLock = new ReentrantReadWriteLock();

    PhotonCamera camera = new PhotonCamera("3534camera");
    AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonVisionCameraTelemetry photonVisionCameraTelemetry = new PhotonVisionCameraTelemetry();

    final Transform3d robotToCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(13.5), 0, Units.inchesToMeters(8.75)),
            new Rotation3d(0, Units.degreesToRadians(-7), 0));

    final PhotonPoseEstimator photonPoseEstimator;

    private static final boolean enabled = true;
    private static PhotonVisionCamera INSTANCE;

    public static Optional<PhotonVisionCamera> createInstance() {
        if (INSTANCE != null) {
            return Optional.of(INSTANCE);
        }
        if (!enabled)
            return Optional.ofNullable(null);
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
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // public void updateOurPose(Consumer<Pose3d> updatePose) {
    // var result = camera.getLatestResult();
    // if (result.hasTargets()) {
    // var target = result.getBestTarget();
    // // Calculate robot's field relative pose
    // var robotPose =
    // PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
    // aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
    // updatePose.accept(robotPose);
    // }
    // }

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
