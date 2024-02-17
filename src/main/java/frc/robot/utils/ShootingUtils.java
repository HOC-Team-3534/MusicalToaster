package frc.robot.utils;

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

    public static Translation2d findVirtualGoalDisplacementFromRobot(double maxError, double maxIterations,
            double maxTimeOfFlight)
            throws Exception {
        return shootWhileMovingSolver.findSolution(getGoalFinalEquation(), maxError, maxIterations, maxTimeOfFlight)
                .getGoalFinalCalculated();
    }

    static GoalFinalEquation getGoalFinalEquation() throws Exception {
        if (goalPositionSupplier == null || robotPositionSupplier == null || robotPositionSupplier == null) {
            throw new Exception("Configure Shoot While Moving before calling findVirtualGoalToAimFor");
        }
        return goalFinalEquation.withGoalPosition(goalPositionSupplier.get())
                .withRobotPosition(robotPositionSupplier.get()).withRobotVelocity(robotSpeedsSupplier.get());
    }

}
