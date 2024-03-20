package frc.robot.utils.pid;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PID4Backlash extends MyProfiledPIDController {

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0); // No output without calling WithFeedforward

    double resetLimit = Double.POSITIVE_INFINITY;

    double prevMeaursement;
    double prevPrevMeasurement;

    public PID4Backlash(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints, double period) {
        super(Kp, Ki, Kd, constraints, period);
    }

    public PID4Backlash(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        this(Kp, Ki, Kd, constraints, 0.020);
    }

    public PID4Backlash withFeedforward(double kS, double kV) {
        this.feedforward = new SimpleMotorFeedforward(kS, kV);
        return this;
    }

    public PID4Backlash withMaxErrorBeforeReset(double resetLimit) {
        this.resetLimit = resetLimit;
        return this;
    }

    public double calculate(double measurement, double goal, TrapezoidProfile.Constraints constraints) {
        if (constraints.maxVelocity != getConstraints().maxVelocity) {
            super.reset(measurement);
        }

        setConstraints(constraints);

        var positionError = measurement - getSetpoint().position;

        // var prevVelocity = (prevMeaursement - prevPrevMeasurement) / getPeriod();
        var velocity = (measurement - prevMeaursement) / getPeriod();

        if (Math.abs(positionError) > this.resetLimit && Math.abs(velocity) < getConstraints().maxVelocity * 0.05) {
            super.reset(measurement);
        }

        // var targetVelocity = 0.0;
        // if (Math.abs(getPositionError()) / getPeriod() / 2.0 >
        // Math.abs(getSetpoint().velocity)
        // && Math.abs(getSetpoint().velocity) < getConstraints().maxVelocity * 0.02) {
        // targetVelocity = super.calculateNoUpdateSetpoint(measurement);
        // } else {
        // targetVelocity = super.calculate(measurement, goal);
        // }

        // this.prevPrevMeasurement = prevMeaursement;
        // this.prevMeaursement = measurement;

        var targetVelocity = super.calculate(measurement, goal);

        return feedforward.calculate(targetVelocity);
    }
}
