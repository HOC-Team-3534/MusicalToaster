package frc.robot.subsystems.intake;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeRequest.IntakeControlRequestParameters;

public class Intake extends SubsystemBase {

    TalonSRX frontBackMotor, leftRightMotor;

    ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
    protected IntakeRequest m_requestToApply = new IntakeRequest.Idle();

    public Intake() {
        frontBackMotor = new TalonSRX(0);
        leftRightMotor = new TalonSRX(0);
    }

    public Command applyRequest(Supplier<IntakeRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void setControl(IntakeRequest request) {
        try {
            m_stateLock.writeLock().lock();

            m_requestToApply = request;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public class IntakeState {
        public double frontBackCurrentDraw;

        public double leftRightCurrentDraw;
    }

    final IntakeState m_cachedState = new IntakeState();

    public class IntakeThread {
        final Thread m_thread;
        volatile boolean m_running;

        private final MedianFilter peakRemover = new MedianFilter(3);
        private final LinearFilter lowPass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        double m_averageLoopTime = 0;

        public IntakeThread() {
            m_thread = new Thread(this::run);

            m_thread.setDaemon(true);
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

        public void run() {
            while (m_running) {

                try {
                    m_stateLock.readLock().lock();

                    lastTime = currentTime;
                    currentTime = Utils.getCurrentTimeSeconds();

                    m_averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

                    m_requestToApply.apply(new IntakeControlRequestParameters(), frontBackMotor, leftRightMotor);

                    m_cachedState.frontBackCurrentDraw = frontBackMotor.getSupplyCurrent();
                    m_cachedState.leftRightCurrentDraw = leftRightMotor.getSupplyCurrent();

                } finally {
                    m_stateLock.writeLock().unlock();
                }

            }
        }
    }

    public IntakeState getState() {
        try {
            m_stateLock.readLock().lock();

            return m_cachedState;
        } finally {
            m_stateLock.readLock().unlock();
        }
    }

}
