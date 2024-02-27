package frc.robot.subsystems.camera;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionCamera extends SubsystemBase {

    final double UpdateFrequency = 50.0;

    ReadWriteLock m_stateLock = new ReentrantReadWriteLock();

    PhotonCamera camera = new PhotonCamera("3534camera");
    AprilTagFieldLayout aprilTagFieldLayout;

    final Transform3d robotToCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.5), 0, Units.inchesToMeters(8)),
            new Rotation3d(0, Units.degreesToRadians(3), 0));

    final PhotonPoseEstimator photonPoseEstimator;

    Supplier<Pose2d> currentPose2dSupplier;
    BiConsumer<Pose3d, Double> visionMeasureConsumer;

    public PhotonVisionCamera(Supplier<Pose2d> currentPose2dSupplier,
            BiConsumer<Pose3d, Double> visionMeasurementConsumer) {
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        this.currentPose2dSupplier = currentPose2dSupplier;
        this.visionMeasureConsumer = visionMeasurementConsumer;
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

    public class CameraState {
        public Transform3d robotToTarget;
        public Pose3d robotFieldPose;
    }

    final CameraState m_cachedState = new CameraState();

    public class CameraThread {
        final Thread m_thread;
        volatile boolean m_running;

        private final MedianFilter peakRemover = new MedianFilter(3);
        private final LinearFilter lowPass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        double m_averageLoopTime = 0;

        public CameraThread() {
            m_thread = new Thread(this::run);

            m_thread.setDaemon(true);
        }

        /**
         * Starts the odometry thread.
         */
        public void start() {
            m_running = true;
            m_thread.start();
        }

        /**
         * Stops the odometry thread.
         */
        public void stop() {
            stop(0);
        }

        /**
         * Stops the odometry thread with a timeout.
         *
         * @param millis The time to wait in milliseconds
         */
        public void stop(long millis) {
            m_running = false;
            try {
                m_thread.join(millis);
            } catch (final InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        public void run() {
            while (m_running) {
                Timer.delay(1.0 / UpdateFrequency);
                try {
                    m_stateLock.readLock().lock();

                    lastTime = currentTime;
                    currentTime = Utils.getCurrentTimeSeconds();

                    photonPoseEstimator.setReferencePose(currentPose2dSupplier.get());
                    var estimatedPose = photonPoseEstimator.update();

                    if (estimatedPose.isPresent()) {
                        var estimate = estimatedPose.get();
                        var targetId = estimate.targetsUsed.get(0).getFiducialId();
                        var targetPose = aprilTagFieldLayout.getTagPose(targetId).get();
                        visionMeasureConsumer.accept(estimate.estimatedPose,
                                estimate.timestampSeconds);
                        m_cachedState.robotToTarget = targetPose.minus(estimate.estimatedPose);
                    }

                    m_averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

                } finally {
                    m_stateLock.writeLock().unlock();
                }
            }
        }
    }

}
