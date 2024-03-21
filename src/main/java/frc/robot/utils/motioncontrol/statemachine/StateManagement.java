package frc.robot.utils.motioncontrol.statemachine;

import edu.wpi.first.wpilibj.Timer;

public class StateManagement {

    private State prevState, currentState, nextState;

    final InputDataAndError inputDataAndError;

    Timer atGoalTimer = new Timer();

    final double AT_GOAL_STEADY_TIME;
    final double BUFFER_WINDOW;

    protected StateManagement(InputDataAndError inputDataAndError, double atGoalSteadyTime, double bufferWindow) {
        this.inputDataAndError = inputDataAndError;
        this.prevState = State.Stationary;
        this.currentState = State.Stationary;
        this.nextState = State.Stationary;

        this.AT_GOAL_STEADY_TIME = atGoalSteadyTime;
        this.BUFFER_WINDOW = bufferWindow;

        this.atGoalTimer.restart();
    }

    protected void manageState() {
        switch (getState()) {
            case Stationary:
                if (goPositive())
                    next(State.AccelPositive);
                if (goNegative())
                    next(State.AccelNegative);
                break;
            case AccelPositive, DecelPositive:
                if (goNegative() || atGoal()) {
                    // if (goingToFastToSimplySwitchToOvershoot())
                    // next(State.AccelNegative);
                    // else
                    next(State.OvershotNegative);
                } else if (goPositive()) {
                    switch (getState()) {
                        case AccelPositive:
                            if (goDecel())
                                next(State.DecelPositive);
                            break;
                        case DecelPositive:
                            if (!goDecel())
                                next(State.AccelPositive);
                            break;
                        default:
                            break;
                    }
                    if (Math.abs(inputDataAndError.getPositionError()) < BUFFER_WINDOW)
                        next(State.OvershotPositive);
                } else {
                    // !!! SHOULD NEVER HAPPEN : EITHER POSITIVE, NEGATIVE, OR AT GOAL
                }
                break;
            case AccelNegative, DecelNegative:
                if (goPositive() || atGoal()) {
                    // if (goingToFastToSimplySwitchToOvershoot())
                    // next(State.AccelPositive);
                    // else
                    next(State.OvershotPositive);
                } else if (goNegative()) {
                    switch (getState()) {
                        case AccelNegative:
                            if (goDecel())
                                next(State.DecelNegative);
                            break;
                        case DecelNegative:
                            if (!goDecel())
                                next(State.AccelNegative);
                            break;
                        default:
                            break;
                    }
                    if (Math.abs(inputDataAndError.getPositionError()) < BUFFER_WINDOW)
                        next(State.OvershotNegative);
                } else {
                    // !!! SHOULD NEVER HAPPEN : EITHER POSITIVE, NEGATIVE, OR AT GOAL
                }
                break;
            case OvershotPositive, OvershotNegative:
                if (Math.abs(inputDataAndError.getPositionError()) > BUFFER_WINDOW) {
                    if (goPositive())
                        next(State.AccelPositive);
                    else if (goNegative())
                        next(State.AccelNegative);
                } else if (getState().equals(State.OvershotPositive) && goNegative())
                    next(State.OvershotNegative);
                else if (getState().equals(State.OvershotNegative) && goPositive())
                    next(State.OvershotPositive);
                break;
        }

        if ((getState().equals(State.OvershotNegative) || getState().equals(State.OvershotPositive))
                && atGoal()) {// && atGoalTimer.hasElapsed(this.AT_GOAL_STEADY_TIME)) {
            next(State.Stationary);
        } else if (getState().equals(State.Stationary)) {
        } else {
            atGoalTimer.restart();
        }

        goToNext();
    }

    private void next(State nextState) {
        this.nextState = nextState;
    }

    private void goToNext() {
        if (!nextState.equals(currentState)) {
            this.prevState = this.currentState;
            this.currentState = this.nextState;
        }
    }

    public State getState() {
        return this.currentState;
    }

    public State getPrevState() {
        return this.prevState;
    }

    public enum State {
        AccelPositive,
        DecelPositive,
        OvershotPositive,
        Stationary,
        OvershotNegative,
        DecelNegative,
        AccelNegative
    }

    private boolean goPositive() {
        return inputDataAndError.goPositive();
    }

    private boolean goNegative() {
        return inputDataAndError.goNegative();
    }

    private boolean goingToFastToSimplySwitchToOvershoot() {
        return inputDataAndError.goingToFastToSimplySwitchToOvershoot();
    }

    private boolean goDecel() {
        return inputDataAndError.goDecel();
    }

    private boolean atGoal() {
        return inputDataAndError.atGoal();
    }
}
