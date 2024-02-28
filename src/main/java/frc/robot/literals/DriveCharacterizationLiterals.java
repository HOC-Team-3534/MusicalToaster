package frc.robot.literals;

public class DriveCharacterizationLiterals {
    // voltage per second increase
    public double quasiasticVoltage = 1.0;
    // duration of test seconds
    public double quasiasticDuration = 4.5;

    public DriveCharacterizationLiterals withQuasiasticVoltage(double voltagePerSecond) {
        this.quasiasticVoltage = voltagePerSecond;
        return this;
    }

    public DriveCharacterizationLiterals withQuasiasticDuration(double durationSeconds) {
        this.quasiasticDuration = durationSeconds;
        return this;
    }
}
