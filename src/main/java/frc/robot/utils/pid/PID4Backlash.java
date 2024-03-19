package frc.robot.utils.pid;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PID4Backlash extends MyProfiledPIDController {

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0); // No output without calling WithFeedforward

    double resetLimit = Double.POSITIVE_INFINITY;

    double prevMeaursement;

    public PID4Backlash(double Kp, double Ki, double Kd, double maxVelocity, double maxAccel, double period) {
        super(Kp, Ki, Kd, new TrapezoidProfile.Constraints(maxVelocity, maxAccel), period);
    }

    public PID4Backlash(double Kp, double Ki, double Kd, double maxVelocity, double maxAccel) {
        this(Kp, Ki, Kd, maxVelocity, maxAccel, 0.020);
    }

    public PID4Backlash withFeedforward(double kS, double kV) {
        this.feedforward = new SimpleMotorFeedforward(kS, kV);
        return this;
    }

    public PID4Backlash withMaxErrorBeforeReset(double resetLimit) {
        this.resetLimit = resetLimit;
        return this;
    }

    @Override
    public double calculate(
            double measurement, TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints) {
        setConstraints(constraints);
        return this.calculate(measurement, goal);
    }

    @Override
    public double calculate(double measurement, double goal) {
        if (Math.abs(measurement - getSetpoint().position) > this.resetLimit) {
            super.reset(measurement);
        }

        var velocity = (measurement - prevMeaursement) / getPeriod();

        var distanceToCover = Math.abs(getSetpoint().position - measurement); // aka position error
        var checkTime = getPeriod() * 2;
        var distanceCanCoverInCheckTime = Math.abs(velocity) * checkTime
                + 0.5 * getConstraints().maxAcceleration * Math.pow(checkTime, 2);
        var ableToReachInCheckTime = distanceToCover < distanceCanCoverInCheckTime;

        double targetVelocity = 0;
        if (!ableToReachInCheckTime) {
            super.calculateNoUpdateSetpoint(measurement);
        } else {
            targetVelocity = super.calculate(measurement, goal);
        }

        this.prevMeaursement = measurement;

        return feedforward.calculate(targetVelocity);
    }
}
