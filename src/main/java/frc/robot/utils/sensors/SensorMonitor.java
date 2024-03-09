package frc.robot.utils.sensors;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Queue;

public class SensorMonitor {
    private Queue<Double> buffer;
    private int bufferSize;
    private double maxDelta;

    public SensorMonitor(int bufferSize, double maxDelta) {
        this.buffer = new ArrayDeque<>(bufferSize);
        this.bufferSize = bufferSize;
        this.maxDelta = maxDelta;
    }

    public SensorMonitor(double seconds, double loopPeriod, double maxDelta) {
        this((int) (seconds / loopPeriod), maxDelta);
    }

    public void addSensorValue(double value) {
        if (buffer.size() >= bufferSize) {
            buffer.poll(); // Remove oldest value
        }
        buffer.offer(value); // Add new value
    }

    public boolean hasSignificantMovement() {
        if (buffer.size() < bufferSize) {
            return false; // Not enough data to determine
        }

        double min = Collections.min(buffer);
        double max = Collections.max(buffer);
        return (max - min) > maxDelta;
    }
}
