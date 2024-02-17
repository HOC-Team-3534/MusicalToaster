package frc.robot.utils.shooting;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Translation2d;

public class SolverSolution {
	final double timeOfFlightGuess;
	double timeOfFlight;
	final Translation2d goalFinalGuess;
	Translation2d goalFinal;
	double error;

	public SolverSolution(double timeOfFlightGuess, Translation2d goalFinalGuess, GoalFinalEquation goalFinalEquation,
			Function<Translation2d, Double> timeOfFlightEquation) {
		this.timeOfFlightGuess = timeOfFlightGuess;
		this.goalFinalGuess = goalFinalGuess;
		this.calculate(goalFinalEquation, timeOfFlightEquation);
	}

	public double getTimeOfFlightGuess() {
		return timeOfFlightGuess;
	}

	public Translation2d getGoalFinalGuess() {
		return goalFinalGuess;
	}

	public double getTimeOfFlightCalculated() {
		return timeOfFlight;
	}

	public Translation2d getGoalFinalCalculated() {
		return goalFinal;
	}

	public double getError() {
		return error;
	}

	void calculate(GoalFinalEquation goalFinalEquation, Function<Translation2d, Double> timeOfFlightEquation) {
		timeOfFlight = timeOfFlightEquation.apply(goalFinalGuess);
		goalFinal = goalFinalEquation.getGoalFinal(timeOfFlightGuess);

		error = Math.abs(timeOfFlightGuess - timeOfFlight);
	}

	@Override
	public String toString() {
		return timeOfFlightGuess + ", " + error;
	}
}
