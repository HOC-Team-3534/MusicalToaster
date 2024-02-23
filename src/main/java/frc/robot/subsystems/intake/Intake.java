package frc.robot.subsystems.intake;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeRequest.IntakeControlRequestParameters;

public class Intake extends SubsystemBase {

    TalonSRX frontBackMotor, leftRightMotor;
    ProximitySensorInput sensors[] = new ProximitySensorInput[4];

    final double UpdateFrequency = 100.0;

    final static double delayNoteInPositionSeconds = 0.5;
    final static double noteNotSeenSeconds = 0.5;

    private IntakeThread intakeThread;

    final IntakeTelemetry intakeTelemetry = new IntakeTelemetry();

    ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
    protected IntakeRequest m_requestToApply = new IntakeRequest.Idle();
    protected IntakeControlRequestParameters m_requestParameters = new IntakeControlRequestParameters();

    public Intake() {
        frontBackMotor = new TalonSRX(19);
        leftRightMotor = new TalonSRX(20);
        leftRightMotor.setInverted(true);
        frontBackMotor.setInverted(true);
        for (int i = 0; i < sensors.length; i++) {
            sensors[i] = new ProximitySensorInput(i);
        }

        intakeThread = new IntakeThread();
        intakeThread.start();
    }

    public enum IntakeDirection {
        Off, In, Out;

        public static double threshold = 0.01;

        public static IntakeDirection getDirection(double percentOutput) {
            if (percentOutput > threshold)
                return In;
            else if (percentOutput < -threshold)
                return Out;
            else
                return Off;
        }
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
        public IntakeState() {
            for (int i = 0; i < noteInPositionTimer.length; i++) {
                noteInPositionTimer[i] = new Timer();
                noteInPositionTimer[i].start();
                noteNotSeenTimer[i] = new Timer();
                noteNotSeenTimer[i].start();
            }
            for (int i = 0; i < intakeDirection.length; i++) {
                intakeDirection[i] = IntakeDirection.Off;
            }
        }

        // 0 is Front, 1 is Left, Back is 2, Right is 3
        public boolean seeingNote[] = new boolean[4];

        private boolean noteInPosition[] = new boolean[4];

        public IntakeDirection intakeDirection[] = new IntakeDirection[4];

        private final Timer noteInPositionTimer[] = new Timer[4];

        private final Timer noteNotSeenTimer[] = new Timer[4];

        public boolean getRawNoteInPositionNoDelay(int i) {
            return noteInPosition[i];
        }

        public boolean getNoteInPosition(int i) {
            return noteInPosition[i] && noteInPositionTimer[i].hasElapsed(delayNoteInPositionSeconds);
        }
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
                Timer.delay(1.0 / UpdateFrequency);
                try {
                    m_stateLock.writeLock().lock();

                    lastTime = currentTime;
                    currentTime = Utils.getCurrentTimeSeconds();

                    m_averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

                    var fbPerc = frontBackMotor.getMotorOutputPercent();
                    var lrPerc = leftRightMotor.getMotorOutputPercent();
                    double percs[] = { fbPerc, lrPerc, fbPerc, lrPerc };

                    for (int i = 0; i < percs.length; i++) {
                        m_cachedState.intakeDirection[i] = IntakeDirection.getDirection(percs[i]);
                    }

                    for (int i = 0; i < sensors.length; i++) {
                        /*
                         * If the note is not already marked in position
                         * and the note is seen while the motor is running in
                         * set note in position to true and reset the delay timer
                         */
                        if (!m_cachedState.noteInPosition[i]
                                && sensors[i].get()
                                && m_cachedState.intakeDirection[i].equals(IntakeDirection.In)) {
                            m_cachedState.noteInPosition[i] = true;
                            m_cachedState.noteInPositionTimer[i].restart();
                        }

                        // If you see the note, reset timer that resets noteInPosition.
                        if (sensors[i].get())
                            m_cachedState.noteNotSeenTimer[i].restart();

                        /*
                         * If note not seen for duration, reset noteInPosition.
                         * This assumes the note can be seen in its resting position in the intake
                         */
                        if (m_cachedState.noteNotSeenTimer[i].hasElapsed(noteNotSeenSeconds))
                            m_cachedState.noteInPosition[i] = false;

                        m_cachedState.seeingNote[i] = sensors[i].get();
                    }

                    intakeTelemetry.telemeterize(m_cachedState);

                    m_requestParameters.intakeState = m_cachedState;

                    m_requestToApply.apply(m_requestParameters, frontBackMotor,
                            leftRightMotor);

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
