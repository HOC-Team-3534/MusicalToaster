package frc.robot.utils.motioncontrol.statemachine;

public class OutputConstraints {
    final protected double maxVelocity, maxAccel, maxDecel, overshootMaxVelocity, initialBacklashMaxVelocity;

    public OutputConstraints(double maxVelocity, double maxAccel, double maxDecel, double overshootMaxVelocity,
            double initialBacklashMaxVelocity) {
        this.maxVelocity = Math.abs(maxVelocity);
        this.maxAccel = Math.abs(maxAccel);
        this.maxDecel = Math.abs(maxDecel);
        this.overshootMaxVelocity = Math.abs(overshootMaxVelocity);
        this.initialBacklashMaxVelocity = Math.abs(initialBacklashMaxVelocity);
    }
}
