package frc.robot.utils;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.shooting.GoalFinalEquation;
import frc.robot.utils.shooting.ShootWhileMovingSolver;

public class ShootingUtils {

    static Supplier<Translation2d> goalPositionSupplier;
    static Supplier<ChassisSpeeds> robotSpeedsSupplier;
    static Supplier<Translation2d> robotPositionSupplier;

    static ShootWhileMovingSolver shootWhileMovingSolver;

    static final GoalFinalEquation goalFinalEquation = new GoalFinalEquation();

    public static void configureShootWhileMoving(Supplier<Translation2d> goalPositionSupplier,
            Supplier<ChassisSpeeds> robotSpeedsSupplier, Supplier<Translation2d> robotPositionSupplier,
            Function<Translation2d, Double> timeOfFlightEquation) {
        ShootingUtils.goalPositionSupplier = goalPositionSupplier;
        ShootingUtils.robotSpeedsSupplier = robotSpeedsSupplier;
        ShootingUtils.robotPositionSupplier = robotPositionSupplier;

        shootWhileMovingSolver = new ShootWhileMovingSolver(timeOfFlightEquation);
    }

    public static Optional<Translation2d> findVirtualGoalDisplacementFromRobot(double maxError, double maxIterations,
            double maxTimeOfFlight) {
        return getGoalFinalEquation().map((goalEquation) -> shootWhileMovingSolver
                .findSolution(goalEquation, maxError, maxIterations, maxTimeOfFlight)
                .getGoalFinalCalculated());
    }

    static Optional<GoalFinalEquation> getGoalFinalEquation() {
        if (goalPositionSupplier == null || robotPositionSupplier == null || robotPositionSupplier == null)
            return Optional.empty();

        var goalPosition = goalPositionSupplier.get();
        if (goalPosition == null)
            return Optional.empty();
        return Optional.of(goalFinalEquation.withGoalPosition(goalPositionSupplier.get())
                .withRobotPosition(robotPositionSupplier.get()).withRobotVelocity(robotSpeedsSupplier.get()));
    }

}
