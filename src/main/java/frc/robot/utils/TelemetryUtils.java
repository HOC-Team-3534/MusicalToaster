package frc.robot.utils;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TelemetryUtils {

    static Optional<PathPlannerPath> displayedPath;

    public static Optional<Trajectory> getTrajectoryIfChanged(Supplier<Optional<PathPlannerPath>> currentPathSupplier) {
        var currentPath = currentPathSupplier.get();

        if (EdgeDetectionUtils.detectChange(displayedPath, currentPath))
            return Optional.of(currentPath.map(path -> TelemetryUtils.getWPILIBTrajectory(path))
                    .orElse(new Trajectory()));
        return Optional.empty();
    }

    private static PathPlannerTrajectory getDisplayTrajectory(PathPlannerPath path) {
        var alliancePath = DriverStation.getAlliance()
                .map(alliance -> alliance.equals(Alliance.Red) ? path.flipPath() : path).orElse(path);
        return alliancePath.getTrajectory(new ChassisSpeeds(),
                alliancePath.getPreviewStartingHolonomicPose().getRotation());
    }

    public static Trajectory getWPILIBTrajectory(PathPlannerPath path) {
        return getWPILIBTrajectory(getDisplayTrajectory(path));
    }

    private static Trajectory getWPILIBTrajectory(PathPlannerTrajectory trajectory) {

        List<Trajectory.State> states = trajectory.getStates().stream()
                .map(state -> new Trajectory.State(state.timeSeconds, state.velocityMps, state.accelerationMpsSq,
                        state.getTargetHolonomicPose(),
                        state.curvatureRadPerMeter))
                .toList();

        return new Trajectory(states);
    }
}
