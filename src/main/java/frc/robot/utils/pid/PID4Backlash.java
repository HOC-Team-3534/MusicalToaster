package frc.robot.utils.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.sensors.SensorMonitor;

public class PID4Backlash extends ProfiledPIDController {

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0); // No output without calling WithFeedforward
    SensorMonitor stillSensorMonitor = new SensorMonitor(1, 0);

    double maxSpeedConsideredStill;
    double outputScaleDuringBacklashAboveStillSpeed = 1.3;

    TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

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

    public PID4Backlash withStillMonitor(double durationWindow, double maxDelta) {
        this.stillSensorMonitor = new SensorMonitor(durationWindow, getPeriod(), maxDelta);
        this.maxSpeedConsideredStill = maxDelta / durationWindow;
        return this;
    }

    @Override
    public double calculate(double measurement, double goal) {
        stillSensorMonitor.addSensorValue(measurement);

        var targetVelocity = super.calculate(measurement, goal);

        if (!stillSensorMonitor.hasSignificantMovement()) {
            if (Math.abs(targetVelocity) > getVelocityDuringBacklash()) {
                targetVelocity = Math.copySign(getVelocityDuringBacklash(), targetVelocity);
            }
            super.reset(measurement);
        }

        this.setpoint = new TrapezoidProfile.State(super.getSetpoint().position, targetVelocity);

        return feedforward.calculate(targetVelocity);
    }

    private double getVelocityDuringBacklash() {
        return this.maxSpeedConsideredStill * this.outputScaleDuringBacklashAboveStillSpeed;
    }

    @Override
    public TrapezoidProfile.State getSetpoint() {
        return this.setpoint;
    }
}
