package frc.robot.utils.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PID4Backlash extends ProfiledPIDController {

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0); // No output without calling WithFeedforward

    double speedWhileInBacklash;

    double prevMeasurement;

    TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    double resetLimit;

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

    public PID4Backlash withSpeedWhileInBacklash(double backlashSpeed) {
        this.speedWhileInBacklash = backlashSpeed;
        return this;
    }

    public PID4Backlash withMaxErrorBeforeReset(double resetLimit) {
        this.resetLimit = resetLimit;
        return this;
    }

    @Override
    public double calculate(double measurement, double goal) {
        var motion = measurement - prevMeasurement;
        // Reset Setpoint for proper profiling when there has not been significant
        // movement and the error is more than can be solved in a second during backlash
        if (Math.abs(measurement - super.getSetpoint().position) > this.resetLimit) {
            super.reset(measurement, motion / getPeriod());
        }

        var targetVelocity = super.calculate(measurement, goal);

        var motionInOppositionToOutput = motion / (targetVelocity) < 0;

        if (motionInOppositionToOutput) {
            if (Math.abs(targetVelocity) > this.speedWhileInBacklash) {
                targetVelocity = Math.copySign(this.speedWhileInBacklash, targetVelocity);
            }
        }

        this.setpoint = new TrapezoidProfile.State(super.getSetpoint().position, targetVelocity);

        this.prevMeasurement = measurement;

        return feedforward.calculate(targetVelocity);
    }

    @Override
    public TrapezoidProfile.State getSetpoint() {
        return this.setpoint;
    }
}
