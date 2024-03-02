package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/** Pathfind and follow the path with a PPHolonomicDriveController */
public class PathfindHolonomic extends PathfindingCommand {

        /**
         * Constructs a new PathfindHolonomic command that will generate a path towards
         * the given pose.
         *
         * @param targetPosition
         *                                   the pose to pathfind to
         * @param rotationFunction
         *                                   the target rotation of the robot based on
         *                                   the current trajectory
         *                                   state
         * @param constraints
         *                                   the path constraints to use while
         *                                   pathfinding
         * @param goalEndVel
         *                                   The goal end velocity when reaching the
         *                                   given pose
         * @param poseSupplier
         *                                   a supplier for the robot's current pose
         * @param currentRobotRelativeSpeeds
         *                                   a supplier for the robot's current robot
         *                                   relative speeds
         * @param output
         *                                   a consumer for the output speeds (field
         *                                   relative if holonomic,
         *                                   robot relative if
         *                                   differential)
         * @param config
         *                                   HolonomicPathFollowerConfig object with the
         *                                   configuration
         *                                   parameters for path
         *                                   following
         * @param requirements
         *                                   the subsystems required by this command
         */
        public PathfindHolonomic(
                        Translation2d targetPosition,
                        Function<PathPlannerTrajectory.State, Rotation2d> rotationFunction,
                        PathConstraints constraints,
                        double goalEndVel,
                        Supplier<Pose2d> poseSupplier,
                        Supplier<ChassisSpeeds> currentRobotRelativeSpeeds,
                        Consumer<ChassisSpeeds> output,
                        HolonomicPathFollowerConfig config,
                        Subsystem... requirements) {
                super(
                                targetPosition,
                                rotationFunction,
                                constraints,
                                goalEndVel,
                                poseSupplier,
                                currentRobotRelativeSpeeds,
                                output,
                                new PPHolonomicDriveController(
                                                config.translationConstants,
                                                config.rotationConstants,
                                                config.period,
                                                config.maxModuleSpeed,
                                                config.driveBaseRadius),
                                config.replanningConfig,
                                requirements);
        }
}
