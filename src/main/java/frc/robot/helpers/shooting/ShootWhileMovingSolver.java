package frc.robot.helpers.shooting;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;

public class ShootWhileMovingSolver {
	int iterations;
	long startedAt, completedAt;

	final GoalFinalEquation goalFinalEquation;
	final Function<Translation2d, Double> timeOfFlightEquation;

	public ShootWhileMovingSolver(GoalFinalEquation goalFinalEquation,
			Function<Translation2d, Double> timeOfFlightEquation) {
		this.goalFinalEquation = goalFinalEquation;
		this.timeOfFlightEquation = timeOfFlightEquation;
	}

	public SolverSolution findSolution(double maxError, double maxIterations, double maxTimeOfFlight) {
		startedAt = System.currentTimeMillis();
		iterations = 0;

		double lowerBound = 0;
		double upperBound = maxTimeOfFlight;

		SolverSolution latestSolution;

		do {
			iterations++;

			double difference = upperBound - lowerBound;
			double midPoint = lowerBound + (difference) / 2.0;
			double midPointSpace = difference / 1e6;

			SolverSolution first = new SolverSolution(midPoint - midPointSpace,
					goalFinalEquation.getGoalFinal(midPoint - midPointSpace), goalFinalEquation, timeOfFlightEquation);
			SolverSolution second = new SolverSolution(midPoint, goalFinalEquation.getGoalFinal(midPoint),
					goalFinalEquation, timeOfFlightEquation);

			if (first.getError() < second.getError()) {
				upperBound = second.getTimeOfFlightGuess();
				latestSolution = first;
			} else {
				lowerBound = first.getTimeOfFlightGuess();
				latestSolution = second;
			}

		} while (latestSolution.error > maxError && iterations < maxIterations);

		completedAt = System.currentTimeMillis();
		return latestSolution;
	}

	public ArrayList<SolverSolution> getTable(double maxTimeOfFlight) {
		ArrayList<SolverSolution> solutions = new ArrayList<>();
		for (double i = 0; i < maxTimeOfFlight; i += (maxTimeOfFlight) / 16.0) {
			SolverSolution currentSolution = new SolverSolution(i, goalFinalEquation.getGoalFinal(i), goalFinalEquation,
					timeOfFlightEquation);
			solutions.add(currentSolution);
		}
		return solutions;
	}

	public int getIterations() {
		return iterations;
	}

	public long getDuration() {
		return completedAt - startedAt;
	}

}
