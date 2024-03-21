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
                return accel();
            case DecelNegative, DecelPositive:
                return decel();
            case OvershotNegative, OvershotPositive:
                return overshoot();
            case Stationary:
                this.outputVelocity = 0;
                return this.outputVelocity;
            default:
                return 0;
        }
    }

    private double overshoot() {
        return Math.copySign(inputDataAndError.atGoal() ? 0 : CONSTRAINTS.overshootMaxVelocity,
                inputDataAndError.getPositionError());
    }

    private double accel() {
        // ACCELERATE IN DIRECTION OF POSITION ERROR
        this.outputVelocity += Math.copySign(accelInc(), inputDataAndError.getPositionError());

        // CAP VELOCITY AT MAX VELOCITY
        if (Math.abs(this.outputVelocity) > CONSTRAINTS.maxVelocity)
            this.outputVelocity = Math.copySign(CONSTRAINTS.maxVelocity, this.outputVelocity);

        adjustVelocitySlowerToMatchSlowerThanExpectedVelocity();

        if (Math.abs(this.outputVelocity) < CONSTRAINTS.overshootMaxVelocity)
            this.outputVelocity = Math.copySign(CONSTRAINTS.overshootMaxVelocity, inputDataAndError.getPositionError());

        return this.outputVelocity;
    }

    private double adjustVelocitySlowerToMatchSlowerThanExpectedVelocity() {
        // EXPECTING TO GO FASTER
        if (MathUtils.sameSign(inputDataAndError.getVelocityError(), this.outputVelocity)
                && Math.abs(inputDataAndError.getVelocityError()) / accelInc() >= 5) {
            // HALVE ERROR IF MORE THAN ACCELERATE IN 2 INC
            this.outputVelocity -= Math.copySign(inputDataAndError.getVelocityError() / 20.0, this.outputVelocity);
        }
        return this.outputVelocity;
    }

    private double accelInc() {
        return CONSTRAINTS.maxAccel * PERIOD;
    }

    private double decel() {

        var prevOutput = this.outputVelocity;

        this.outputVelocity -= Math.copySign(decelInc(), this.outputVelocity);

        if (MathUtils.oppositeSign(prevOutput, this.outputVelocity))
            this.outputVelocity = Math.copySign(CONSTRAINTS.overshootMaxVelocity, prevOutput);

        return this.outputVelocity;
    }

    private double decelInc() {
        return CONSTRAINTS.maxDecel * PERIOD;
    }

}
