package frc.robot.subsystems;

import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionCamera extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("3534camera");
    AprilTagFieldLayout aprilTagFieldLayout;
    final Transform3d cameraToRobot = new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.5), 0, Units.inchesToMeters(8)),
            new Rotation3d(0, Units.degreesToRadians(3), 0));

    public PhotonVisionCamera() {
        aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        switch (DriverStation.getAlliance().get()) {
            case Blue:
                aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
                break;
            case Red:
                aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
                break;
            default:
                break;
        }
    }

    public void updateOurPose(Consumer<Pose3d> updatePose) {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            // Calculate robot's field relative pose
            var robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                    aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
            updatePose.accept(robotPose);
        }
    }

    // TODO write code to get the desired end position for DTM to Grid Pose
    // public Pose3d getGridPose(){

    // }
}
