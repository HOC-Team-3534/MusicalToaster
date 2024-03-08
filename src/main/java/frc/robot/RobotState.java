package frc.robot;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Drive.FIELD_DIMENSIONS;
import frc.robot.ControllerInputs.BTN;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;

public class RobotState {
    int grabNoteIndex = -1;
    boolean activelyGrabbing;
    boolean noteLoaded;
    boolean climbing;
    boolean resetingClimber;
    boolean isExtaking;

    public boolean isActivelyGrabbing() {
        return this.activelyGrabbing;
    }

    public void setActivelyGrabbing(boolean activelyGrabbing) {
        this.activelyGrabbing = activelyGrabbing;
    }

    public boolean isNoteLoaded() {
        return this.noteLoaded;
    }

    public void setNoteLoaded(boolean noteLoaded) {
        this.noteLoaded = noteLoaded;
    }

    public int getGrabNoteIndex() {
        return this.grabNoteIndex;
    }

    public void setGrabNoteIndex(int grabNoteIndex) {
        this.grabNoteIndex = grabNoteIndex;
    }

    public boolean isNoteInRobot() {
        return grabNoteIndex != -1 || noteLoaded;
    }

    public void setClimbing() {
        this.climbing = true;
    }

    public void resetClimbing() {
        this.climbing = false;
    }

    public boolean isClimbing() {
        return this.climbing;
    }

    public void setResetingClimber() {
        this.resetingClimber = true;
    }

    public void resetResetingClimber() {
        this.resetingClimber = false;
    }

    public boolean isResetingClimber() {
        return this.resetingClimber;
    }

    public void setExtaking() {
        this.isExtaking = true;
    }

    public void resetExtaking() {
        this.isExtaking = false;
    }

    public boolean isExtaking() {
        return this.isExtaking;
    }

    public static Optional<Pose2d> getPose() {
        return CommandSwerveDrivetrain.getInstance()
                .map((drivetrain) -> drivetrain.getState().Pose);
    }

    public static Optional<Rotation2d> getPoseRotation() {
        return CommandSwerveDrivetrain.getInstance()
                .map((drivetrain) -> drivetrain.getState().Pose.getRotation());
    }

    public static boolean isValidShootPosition() {
        var speeds = CommandSwerveDrivetrain.getInstance().map(drivetrain -> drivetrain.getChassisSpeeds())
                .orElse(new ChassisSpeeds());
        var drivingSlow = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getNorm() < 0.25;
        return isWithinWingAKABehindAllianceLine() && !isTiltForcedFlat() && drivingSlow;
    }

    public static boolean isBeyondWing() {
        return !isWithinWingAKABehindAllianceLine();
    }

    public static boolean isWithinWingAKABehindAllianceLine() {
        return RobotState.getPose().map((pose) -> {
            var x = pose.getX();
            var alliance = DriverStation.getAlliance().get();
            return alliance.equals(Alliance.Blue)
                    ? x < FIELD_DIMENSIONS.CENTER_OF_FIELD.minus(FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER)
                            .getX()
                    : x > FIELD_DIMENSIONS.CENTER_OF_FIELD.plus(FIELD_DIMENSIONS.OFFSET_ALLIANCE_LINE_FROM_CENTER)
                            .getX();
        }).orElse(true);
    }

    public static boolean isTiltForcedFlat() {
        return BTN.TiltFlat.get() || RobotContainer.getRobotState().isClimbing();
    }

    public static void seedFieldRelativeToInitalPositionIfNoCameraUpdates(List<PathPlannerPath> paths) {
        CommandSwerveDrivetrain.getInstance().ifPresent((drivetrain) -> {
            if (drivetrain.getTimeSincePoseUpdated() > 5.0) {
                var firstPath = DriverStation.getAlliance()
                        .map(alliance -> alliance.equals(Alliance.Red) ? paths.get(0).flipPath()
                                : paths.get(0))
                        .orElse(paths.get(0));
                drivetrain.seedFieldRelative(firstPath.getPreviewStartingHolonomicPose());
            }
        });
    }
}
