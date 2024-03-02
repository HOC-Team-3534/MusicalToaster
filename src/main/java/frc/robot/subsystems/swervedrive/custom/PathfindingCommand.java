package frc.robot.subsystems.swervedrive.custom;

import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/** Base pathfinding command */
public class PathfindingCommand extends Command {
    private static int instances = 0;

    private final Timer timer = new Timer();
    private double goalEndVel;
    private final Translation2d targetPosition;
    private final Function<PathPlannerTrajectory.State, Rotation2d> rotationFunction;
    private final PathConstraints constraints;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final Consumer<ChassisSpeeds> output;
    private final PathFollowingController controller;
    private final ReplanningConfig replanningConfig;

    private PathPlannerPath currentPath;
    private PathPlannerTrajectory currentTrajectory;

    private double timeOffset = 0;

    /**
     * Constructs a new base pathfinding command that will generate a path towards
     * the given pose.
     *
     * @param targetPosition
     *                            the position to pathfind to
     * @param rotationFunction
     *                            the target rotation of the robot based on the
     *                            current trajectory
     *                            state
     * @param constraints
     *                            the path constraints to use while pathfinding
     * @param goalEndVel
     *                            The goal end velocity when reaching the target
     *                            pose
     * @param poseSupplier
     *                            a supplier for the robot's current pose
     * @param speedsSupplier
     *                            a supplier for the robot's current robot relative
     *                            speeds
     * @param outputRobotRelative
     *                            a consumer for the output speeds (robot relative)
     * @param controller
     *                            Path following controller that will be used to
     *                            follow the path
     * @param replanningConfig
     *                            Path replanning configuration
     * @param requirements
     *                            the subsystems required by this command
     */
    public PathfindingCommand(
            Translation2d targetPosition,
            Function<PathPlannerTrajectory.State, Rotation2d> rotationFunction,
            PathConstraints constraints,
            double goalEndVel,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<ChassisSpeeds> outputRobotRelative,
            PathFollowingController controller,
            ReplanningConfig replanningConfig,
            Subsystem... requirements) {
        addRequirements(requirements);

        Pathfinding.ensureInitialized();

        this.targetPosition = targetPosition;
        // this.originalTargetPostion =
        this.rotationFunction = rotationFunction;
        this.goalEndVel = goalEndVel;
        this.constraints = constraints;
        this.controller = controller;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.output = outputRobotRelative;
        this.replanningConfig = replanningConfig;

        instances++;
        HAL.report(tResourceType.kResourceType_PathFindingCommand, instances);
    }

    @Override
    public void initialize() {
        currentTrajectory = null;
        timeOffset = 0;

        Pose2d currentPose = poseSupplier.get();

        controller.reset(currentPose, speedsSupplier.get());

        if (currentPose.getTranslation().getDistance(targetPosition) < 0.5) {
            output.accept(new ChassisSpeeds());
            this.cancel();
        } else {
            Pathfinding.setStartPosition(currentPose.getTranslation());
            Pathfinding.setGoalPosition(targetPosition);
        }
    }

    @Override
    public void execute() {
        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        PathPlannerLogging.logCurrentPose(currentPose);
        PPLibTelemetry.setCurrentPose(currentPose);

        // Skip new paths if we are close to the end
        boolean skipUpdates = currentTrajectory != null
                && currentPose
                        .getTranslation()
                        .getDistance(currentTrajectory.getEndState().positionMeters) < 2.0;

        if (!skipUpdates && Pathfinding.isNewPathAvailable()) {
            currentPath = Pathfinding.getCurrentPath(constraints, new GoalEndState(goalEndVel, new Rotation2d()));

            if (currentPath != null) {
                currentTrajectory = new PathPlannerTrajectory(currentPath, currentSpeeds, currentPose.getRotation());

                // Find the two closest states in front of and behind robot
                int closestState1Idx = 0;
                int closestState2Idx = 1;
                while (closestState2Idx < currentTrajectory.getStates().size() - 1) {
                    double closest2Dist = currentTrajectory
                            .getState(closestState2Idx).positionMeters
                            .getDistance(currentPose.getTranslation());
                    double nextDist = currentTrajectory
                            .getState(closestState2Idx + 1).positionMeters
                            .getDistance(currentPose.getTranslation());
                    if (nextDist < closest2Dist) {
                        closestState1Idx++;
                        closestState2Idx++;
                    } else {
                        break;
                    }
                }

                // Use the closest 2 states to interpolate what the time offset should be
                // This will account for the delay in pathfinding
                var closestState1 = currentTrajectory.getState(closestState1Idx);
                var closestState2 = currentTrajectory.getState(closestState2Idx);

                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds,
                        currentPose.getRotation());
                Rotation2d currentHeading = new Rotation2d(
                        fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
                Rotation2d headingError = currentHeading.minus(closestState1.heading);
                boolean onHeading = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 1.0
                        || Math.abs(headingError.getDegrees()) < 45;

                // Replan the path if our heading is off
                if (onHeading || !replanningConfig.enableInitialReplanning) {
                    double d = closestState1.positionMeters.getDistance(closestState2.positionMeters);
                    double t = (currentPose.getTranslation().getDistance(closestState1.positionMeters)) / d;
                    t = MathUtil.clamp(t, 0.0, 1.0);

                    timeOffset = GeometryUtil.doubleLerp(closestState1.timeSeconds, closestState2.timeSeconds, t);

                    // If the robot is stationary and at the start of the path, set the time offset
                    // to the
                    // next loop
                    // This can prevent an issue where the robot will remain stationary if new paths
                    // come in
                    // every loop
                    if (timeOffset <= 0.02
                            && Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.1) {
                        timeOffset = 0.02;
                    }
                } else {
                    currentPath = currentPath.replan(currentPose, currentSpeeds);
                    currentTrajectory = new PathPlannerTrajectory(currentPath, currentSpeeds,
                            currentPose.getRotation());

                    timeOffset = 0;
                }

                PathPlannerLogging.logActivePath(currentPath);
                PPLibTelemetry.setCurrentPath(currentPath);
            }

            timer.reset();
            timer.start();
        }

        if (currentTrajectory != null) {
            PathPlannerTrajectory.State targetState = currentTrajectory.sample(timer.get() + timeOffset);

            if (replanningConfig.enableDynamicReplanning) {
                double previousError = Math.abs(controller.getPositionalError());
                double currentError = currentPose.getTranslation().getDistance(targetState.positionMeters);

                if (currentError >= replanningConfig.dynamicReplanningTotalErrorThreshold
                        || currentError - previousError >= replanningConfig.dynamicReplanningErrorSpikeThreshold) {
                    replanPath(currentPose, currentSpeeds);
                    timer.reset();
                    timeOffset = 0.0;
                    targetState = currentTrajectory.sample(0);
                }
            }

            targetState.targetHolonomicRotation = rotationFunction.apply(targetState);

            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

            double currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

            PPLibTelemetry.setCurrentPose(currentPose);
            PathPlannerLogging.logCurrentPose(currentPose);

            if (controller.isHolonomic()) {
                PPLibTelemetry.setTargetPose(targetState.getTargetHolonomicPose());
                PathPlannerLogging.logTargetPose(targetState.getTargetHolonomicPose());
            } else {
                PPLibTelemetry.setTargetPose(targetState.getDifferentialPose());
                PathPlannerLogging.logTargetPose(targetState.getDifferentialPose());
            }

            PPLibTelemetry.setVelocities(
                    currentVel,
                    targetState.velocityMps,
                    currentSpeeds.omegaRadiansPerSecond,
                    targetSpeeds.omegaRadiansPerSecond);
            PPLibTelemetry.setPathInaccuracy(controller.getPositionalError());

            output.accept(targetSpeeds);
        }
    }

    @Override
    public boolean isFinished() {
        if (currentTrajectory != null) {
            return timer.hasElapsed(currentTrajectory.getTotalTimeSeconds() - timeOffset);
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        // Only output 0 speeds when ending a path that is supposed to stop, this allows
        // interrupting
        // the command to smoothly transition into some auto-alignment routine
        if (!interrupted && goalEndVel < 0.1) {
            output.accept(new ChassisSpeeds());
        }

        PathPlannerLogging.logActivePath(null);
    }

    private void replanPath(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        PathPlannerPath replanned = currentPath.replan(currentPose, currentSpeeds);
        currentTrajectory = replanned.getTrajectory(currentSpeeds, currentPose.getRotation());
        PathPlannerLogging.logActivePath(replanned);
        PPLibTelemetry.setCurrentPath(replanned);
    }
}
