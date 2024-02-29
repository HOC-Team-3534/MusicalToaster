package frc.robot.subsystems;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;

public abstract class SubsystemThread {
    final Thread m_thread;
    volatile boolean m_running;

    private final double updateFrequency;

    private final MedianFilter peakRemover = new MedianFilter(3);
    private final LinearFilter lowPass = LinearFilter.movingAverage(50);
    private double lastTime = 0;
    private double currentTime = 0;
    double m_averageLoopTime = 0;

    ReadWriteLock m_stateLock = new ReentrantReadWriteLock();

    public SubsystemThread(double updateFrequency) {
        m_thread = new Thread(this::_run);

        m_thread.setDaemon(true);

        this.updateFrequency = updateFrequency;
    }

    /**
     * Starts the odometry thread.
     */
    public void start() {
        m_running = true;
        m_thread.start();
    }

    /**
     * Stops the odometry thread.
     */
    public void stop() {
        stop(0);
    }

    /**
     * Stops the odometry thread with a timeout.
     *
     * @param millis The time to wait in milliseconds
     */
    public void stop(long millis) {
        m_running = false;
        try {
            m_thread.join(millis);
        } catch (final InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    private void _run() {
        while (m_running) {
            Timer.delay(1.0 / updateFrequency);
            try {
                m_stateLock.writeLock().lock();

                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();

                m_averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

                run();
            } finally {
                m_stateLock.writeLock().unlock();
            }
        }
    }

    public abstract void run();

    public Lock writeLock() {
        return m_stateLock.writeLock();
    }

    public Lock readLock() {
        return m_stateLock.readLock();
    }
}