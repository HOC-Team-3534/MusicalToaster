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
    DigitalInput sensors[] = new DigitalInput[4];

    final double UpdateFrequency = 100.0;

    final static double delayNoteInPositionSeconds = 0.25;

    final IntakeTelemetry intakeTelemetry = new IntakeTelemetry();

    ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
    protected IntakeRequest m_requestToApply = new IntakeRequest.Idle();
    protected IntakeControlRequestParameters m_requestParameters = new IntakeControlRequestParameters();

    public Intake() {
        frontBackMotor = new TalonSRX(19);
        leftRightMotor = new TalonSRX(20);
        for (int i = 0; i < sensors.length; i++) {
            sensors[i] = new DigitalInput(i);
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
            }
        }

        public double frontBackCurrentDraw;

        public double leftRightCurrentDraw;

        // 0 is Front, 1 is Left, Back is 2, Right is 3
        public boolean seeingNote[] = new boolean[4];

        public int sensorRisingEdges[] = new int[4];

        private boolean noteInPosition[] = new boolean[4];

        private final Timer noteInPositionTimer[] = new Timer[4];

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
                    m_stateLock.readLock().lock();

                    lastTime = currentTime;
                    currentTime = Utils.getCurrentTimeSeconds();

                    m_averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

                    m_cachedState.frontBackCurrentDraw = frontBackMotor.getSupplyCurrent();
                    m_cachedState.leftRightCurrentDraw = leftRightMotor.getSupplyCurrent();

                    var frontBackMotorOn = Math.abs(frontBackMotor.getMotorOutputPercent()) > 0.01;
                    var leftRightMotorOn = Math.abs(leftRightMotor.getMotorOutputPercent()) > 0.01;

                    boolean[] intakesOn = { frontBackMotorOn, leftRightMotorOn, frontBackMotorOn, leftRightMotorOn };

                    // TODO Feed seeing note and not in position off current

                    for (int i = 0; i < sensors.length; i++) {
                        if (!m_cachedState.seeingNote[i] && sensors[i].get()
                                && (intakesOn[i] || RobotContainer.isActivelyIndexingFromIntake())) {
                            m_cachedState.sensorRisingEdges[i]++;
                            m_cachedState.noteInPositionTimer[i].reset();
                        }

                        m_cachedState.noteInPosition[i] = m_cachedState.sensorRisingEdges[i] % 2 > 0;

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
