package frc.robot.helpers.shooting;;

import java.util.function.Function;

public class SolverSolution {
	final double timeOfFlightGuess;
	double timeOfFlight;
	final Vector2D goalFinalGuess;
	Vector2D goalFinal;
	double error;

	public SolverSolution(double timeOfFlightGuess, Vector2D goalFinalGuess, GoalFinalEquation goalFinalEquation,
			Function<Vector2D, Double> timeOfFlightEquation) {
		this.timeOfFlightGuess = timeOfFlightGuess;
		this.goalFinalGuess = goalFinalGuess;
		this.calculate(goalFinalEquation, timeOfFlightEquation);
	}

	public double getTimeOfFlightGuess() {
		return timeOfFlightGuess;
	}

	public Vector2D getGoalFinalGuess() {
		return goalFinalGuess;
	}

	public double getTimeOfFlightCalculated() {
		return timeOfFlight;
	}

	public Vector2D getGoalFinalCalculated() {
		return goalFinal;
	}

	public double getError() {
		return error;
	}

	void calculate(GoalFinalEquation goalFinalEquation, Function<Vector2D, Double> timeOfFlightEquation) {
		timeOfFlight = timeOfFlightEquation.apply(goalFinalGuess);
		goalFinal = goalFinalEquation.getGoalFinal(timeOfFlightGuess);

		error = Math.abs(timeOfFlightGuess - timeOfFlight);
	}

	@Override
	public String toString() {
		return timeOfFlightGuess + ", " + error;
	}
}
