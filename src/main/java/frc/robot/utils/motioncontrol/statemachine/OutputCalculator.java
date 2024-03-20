package frc.robot.utils.motioncontrol.statemachine;

import frc.robot.utils.MathUtils;

public class OutputCalculator {

    final OutputConstraints CONSTRAINTS;

    final StateManagement stateManagement;
    final InputDataAndError inputDataAndError;

    double outputVelocity;
    final double PERIOD;

    protected OutputCalculator(OutputConstraints constraints, StateManagement stateManagement,
            InputDataAndError inputDataAndError) {
        this(constraints, stateManagement, inputDataAndError, 0.020);
    }

    protected OutputCalculator(OutputConstraints constraints, StateManagement stateManagement,
            InputDataAndError inputDataAndError, double period) {
        this.CONSTRAINTS = constraints;
        this.stateManagement = stateManagement;
        this.inputDataAndError = inputDataAndError;
        this.PERIOD = period;
    }

    protected double calculateOutputVelocity() {
        switch (this.stateManagement.getState()) {
            case AccelNegative, AccelPositive:
                accel();
                return adjustVelocitySlowerToMatchSlowerThanExpectedVelocity();
            case DecelNegative, DecelPositive:
                return decel();
            case OvershotNegative:
                return inputDataAndError.atGoal() ? 0 : -CONSTRAINTS.overshootMaxVelocity;
            case OvershotPositive:
                return inputDataAndError.atGoal() ? 0 : CONSTRAINTS.overshootMaxVelocity;
            case Stationary:
                return 0;
            default:
                return 0;
        }
    }

    private double adjustVelocitySlowerToMatchSlowerThanExpectedVelocity() {
        // EXPECTING TO GO FASTER
        if (MathUtils.sameSign(inputDataAndError.getVelocityError(), this.outputVelocity)
                && Math.abs(inputDataAndError.getVelocityError()) / accelInc() >= 2) {
            // HALVE ERROR IF MORE THAN ACCELERATE IN 2 INC
            this.outputVelocity -= Math.copySign(inputDataAndError.getVelocityError() / 2.0, this.outputVelocity);
        }
        return this.outputVelocity;
    }

    private double accel() {
        // ACCELERATE IN DIRECTION OF ERROR IF STATIONARY
        // OTHERWISE, ACCELERATE FURTHER IN DIRECTION OF POSITION ERROR
        this.outputVelocity += Math.copySign(accelInc(), inputDataAndError.getPositionError());

        // CAP VELOCITY AT MAX VELOCITY
        if (Math.abs(this.outputVelocity) > CONSTRAINTS.maxVelocity)
            this.outputVelocity = Math.copySign(CONSTRAINTS.maxVelocity, this.outputVelocity);

        return this.outputVelocity;
    }

    private double accelInc() {
        return CONSTRAINTS.maxAccel * PERIOD;
    }

    private double decel() {

        this.outputVelocity = (Math.abs(this.outputVelocity) <= CONSTRAINTS.overshootMaxVelocity)
                // DECREASING VELOCITY WOULD GO DOWN TO 0 OR PAST 0 OR JUST TOO LOW
                ? Math.copySign(CONSTRAINTS.overshootMaxVelocity, this.outputVelocity)
                // DECREASE VELOCITY, KNOWING IT WILL NOT GO IN OPPOSITE DIRECTION
                : this.outputVelocity - Math.copySign(decelInc(), this.outputVelocity);
        return this.outputVelocity;
    }

    private double decelInc() {
        return CONSTRAINTS.maxDecel * PERIOD;
    }

}
