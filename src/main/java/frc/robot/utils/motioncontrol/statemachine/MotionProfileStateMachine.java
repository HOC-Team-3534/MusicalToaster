package frc.robot.utils.motioncontrol.statemachine;

import java.util.function.Supplier;

public class MotionProfileStateMachine {

    // TODO BUILD IN DETECTION FOR MAJOR OCSCILATION
    // -> THEN FORCE DECEL TO 0
    // -> THEN GO TO STATIONARY LIKE NORMAL (WHICH WILL RESTART THE PROCESS OF
    // ACCELERATION TOWARDS GOAL)

    private final StateManagement stateManagement;
    private final InputDataAndError inputDataAndError;
    private final OutputCalculator outputCalculator;

    final Supplier<Double> measurementSupplier;

    final OutputConstraints CONSTRAINTS;

    double outputVelocity;

    public MotionProfileStateMachine(Supplier<Double> measurementSupplier, double goalTolerance,
            double atGoalSteadyTime,
            double decelBufferWindow,
            OutputConstraints constraints) {
        this.measurementSupplier = measurementSupplier;

        this.CONSTRAINTS = constraints;
        this.inputDataAndError = new InputDataAndError(goalTolerance, decelBufferWindow, constraints);
        this.stateManagement = new StateManagement(inputDataAndError, atGoalSteadyTime, goalTolerance);
        this.outputCalculator = new OutputCalculator(constraints, stateManagement, inputDataAndError);
    }

    public double determineStateAndCalculateVelocity(double goalPosition) {
        this.inputDataAndError.updateInputs(measurementSupplier.get(), goalPosition, this.outputVelocity);

        this.stateManagement.manageState();

        this.outputVelocity = this.outputCalculator.calculateOutputVelocity();

        return this.outputVelocity;
    }

    public StateManagement.State getState() {
        return this.stateManagement.getState();
    }

    public double getVelocityError() {
        return this.inputDataAndError.getVelocityError();
    }

    public double getPositionError() {
        return this.inputDataAndError.getPositionError();
    }

}
