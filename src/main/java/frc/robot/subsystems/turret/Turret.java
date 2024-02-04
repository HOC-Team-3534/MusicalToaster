package frc.robot.subsystems.turret;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeRequest;
import frc.robot.subsystems.turret.TurretRequest.TurretControlRequestParameters;

public class Turret extends SubsystemBase {
    Supplier<Rotation2d> robotHeading;

    TalonFX rightShooterMotor, leftShooterMotor, rotateMotor, tiltMotor;
    TalonSRX rollerMotor;
    DigitalInput sensor;

    private final double UpdateFrequency = 150.0;

    ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
    protected TurretRequest m_requestToApply = new TurretRequest.Idle();
    protected ShooterRequest m_requestToApplyToShooter = new ShooterRequest.Idle();
    protected TurretControlRequestParameters m_requestParameters = new TurretControlRequestParameters();

    public Turret(Supplier<Rotation2d> robotHeading) {
        super();
        rightShooterMotor = new TalonFX(0);
        leftShooterMotor = new TalonFX(0);
        rotateMotor = new TalonFX(0);
        tiltMotor = new TalonFX(0);
        rollerMotor = new TalonSRX(0);
        sensor = new DigitalInput(4);

        leftShooterMotor.setControl(new Follower(0, true));// TODO Set master ID to right shooter
        // TODO Configuration of gear ratio and set them, add analog sensor for position
        // on motor,

        this.robotHeading = robotHeading;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Command applyRequest(Supplier<TurretRequest> requestSupplier,
            Supplier<ShooterRequest> shooterRequestSupplier) {
        return run(() -> this.setControl(requestSupplier.get(), shooterRequestSupplier.get()));
    }

    private void setControl(TurretRequest request, ShooterRequest shooterRequest) {
        try {
            m_stateLock.writeLock().lock();

            m_requestToApply = request;
            m_requestToApplyToShooter = shooterRequest;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    public class TurretState {
        public Rotation2d azimuth;

        public Rotation2d elevation;

        public boolean noteLoaded;

    }

    final TurretState m_cachedState = new TurretState();

    public class TurretThread {
        final Thread m_thread;
        volatile boolean m_running;

        private final MedianFilter peakRemover = new MedianFilter(3);
        private final LinearFilter lowPass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        double m_averageLoopTime = 0;

        public TurretThread() {
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

                    m_cachedState.azimuth = Rotation2d.fromRotations(rotateMotor.getPosition().getValueAsDouble());

                    m_cachedState.elevation = Rotation2d.fromRotations(tiltMotor.getPosition().getValueAsDouble());

                    m_cachedState.noteLoaded = sensor.get() || m_cachedState.noteLoaded;

                    m_requestParameters.turretState = m_cachedState;

                    m_requestToApply.apply(m_requestParameters, rotateMotor, tiltMotor, rollerMotor);
                    m_requestToApplyToShooter.apply(m_requestParameters, rightShooterMotor);
                } finally {
                    m_stateLock.writeLock().unlock();
                }
            }
        }
    }

    public Rotation2d getRobotRelativeAngle() {
        return null;
    }

    public Rotation2d getFieldRelativeAngle() {
        return getRobotRelativeAngle().plus(robotHeading.get());
    }

}
