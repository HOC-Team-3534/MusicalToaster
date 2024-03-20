package frc.robot.utils.motioncontrol.statemachine;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.MathUtils;

public class InputDataAndError {

    double prevGoalPosition, prevMeasurement, prevLoopTimestamp;
    double currentGoalPosition, currentMeasurement, currentLoopTimestamp;

    final double GOAL_TOLERANCE;

    final OutputConstraints CONSTRAINTS;

    double expectedOutputVelocity;

    protected InputDataAndError(double goalTolerance, OutputConstraints constraints) {
        this.GOAL_TOLERANCE = goalTolerance;
        this.CONSTRAINTS = constraints;
    }

    protected void updateInputs(double measurement, double goalPosition, double expectedOutputVelocity) {
        this.prevGoalPosition = this.currentGoalPosition;
        this.prevMeasurement = this.currentMeasurement;
        this.prevLoopTimestamp = this.currentLoopTimestamp;
        this.currentMeasurement = measurement;
        this.currentGoalPosition = goalPosition;
        this.currentLoopTimestamp = Timer.getFPGATimestamp();
        this.expectedOutputVelocity = expectedOutputVelocity;
    }

    protected double getPositionError() {
        return this.currentGoalPosition - this.currentMeasurement;
    }

    protected boolean goPositive() {
        return getPositionError() > 0 && !atGoal();
    }

    protected boolean goNegative() {
        return getPositionError() < 0 && !atGoal();
    }

    protected boolean atGoal() {
        return Math.abs(getPositionError()) <= GOAL_TOLERANCE;
    }

    protected boolean goDecel() {
        if (MathUtils.oppositeSign(getVelocity(), getPositionError()))
            return false;
        var decelTime = Math.abs(getVelocity() / CONSTRAINTS.maxDecel);
        var distanceToDecel = Math.abs(getVelocity()) * decelTime / 2.0;
        var distanceRemaining = Math.abs(getPositionError());
        return distanceToDecel >= distanceRemaining;
    }

    private double getVelocity() {
        return (this.currentMeasurement - this.prevMeasurement) / getPrevLoopTime();
    }

    private double getPrevLoopTime() {
        return this.currentLoopTimestamp - this.prevLoopTimestamp;
    }

    protected boolean goingToFastToSimplySwitchToOvershoot() {
        return Math.abs(getVelocity()) > 2 * CONSTRAINTS.overshootMaxVelocity;
    }

    protected double getVelocityError() {
        return this.expectedOutputVelocity - getVelocity();
    }

}
