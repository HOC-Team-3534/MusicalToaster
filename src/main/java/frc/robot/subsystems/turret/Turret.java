package frc.robot.subsystems.turret;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretRequest.TurretControlRequestParameters;
import frc.robot.utils.ShootingUtils;
import frc.robot.utils.shooting.QuadraticEquation;

public class Turret extends SubsystemBase {
    final QuadraticEquation timeOfFlightEquation = new QuadraticEquation().withA(0).withB(0).withC(0);

    TalonFX rightShooterMotor, leftShooterMotor, rotateMotor, tiltMotor;
    TalonSRX rollerMotor;
    DigitalInput sensor;

    private final double UpdateFrequency = 150.0;

    ReadWriteLock m_stateLock = new ReentrantReadWriteLock();
    protected TurretRequest m_requestToApply = new TurretRequest.Idle();
    protected ShooterRequest m_requestToApplyToShooter = new ShooterRequest.Idle();
    protected TurretControlRequestParameters m_requestParameters = new TurretControlRequestParameters();
    final Supplier<Rotation2d> m_bottomEncoderRotationSupplier;

    public Turret(Supplier<SwerveDriveState> swerveDriveStateSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        super();
        rightShooterMotor = new TalonFX(0);
        leftShooterMotor = new TalonFX(0);
        rotateMotor = new TalonFX(0);
        tiltMotor = new TalonFX(0);
        rollerMotor = new TalonSRX(0);
        sensor = new DigitalInput(4);

        /*
         * 
         */
        TalonFXConfiguration cfgRotate = new TalonFXConfiguration();

        MotionMagicConfigs mmRotate = cfgRotate.MotionMagic;
        mmRotate.MotionMagicCruiseVelocity = .5;
        mmRotate.MotionMagicAcceleration = 5;
        mmRotate.MotionMagicJerk = 30;

        Slot0Configs slot0Rotate = cfgRotate.Slot0;
        slot0Rotate.kP = 100;
        slot0Rotate.kI = 0;
        slot0Rotate.kV = 8.7184;// TODO Tune these values
        slot0Rotate.kS = 0.6733;

        FeedbackConfigs fdbRotate = cfgRotate.Feedback;
        fdbRotate.SensorToMechanismRatio = 125;

        StatusCode statusRotate = StatusCode.StatusCodeNotInitialized;

        /*
         * 
         */
        TalonFXConfiguration cfgTilt = new TalonFXConfiguration();

        MotionMagicConfigs mmTilt = cfgTilt.MotionMagic;
        mmTilt.MotionMagicCruiseVelocity = .15;
        mmTilt.MotionMagicAcceleration = 2.5;
        mmTilt.MotionMagicJerk = 30;

        Slot0Configs slot0Tilt = cfgTilt.Slot0;
        slot0Tilt.kP = 100;
        slot0Tilt.kI = 0;
        slot0Tilt.kV = 8.7184;// TODO Tune these values
        slot0Tilt.kS = 0.6733;

        FeedbackConfigs fdbTilt = cfgRotate.Feedback;
        fdbTilt.SensorToMechanismRatio = 300;

        StatusCode statusTilt = StatusCode.StatusCodeNotInitialized;
        /* 
         * 
        */

        TalonFXConfiguration cfgShooterMotor = new TalonFXConfiguration();

        Slot0Configs slot0ShooterMotor = cfgShooterMotor.Slot0;
        slot0ShooterMotor.kP = 10;
        slot0ShooterMotor.kI = 0;
        slot0ShooterMotor.kV = 0;
        slot0ShooterMotor.kS = 0;

        FeedbackConfigs velocityConfigsShooterMotor = cfgShooterMotor.Feedback;
        velocityConfigsShooterMotor.SensorToMechanismRatio = 1;

        var shooterCurrentConfig = cfgShooterMotor.CurrentLimits;
        shooterCurrentConfig.SupplyCurrentLimit = 30;

        StatusCode statusShooterL = StatusCode.StatusCodeNotInitialized;
        StatusCode statusShooterR = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; i++) {
            statusRotate = rotateMotor.getConfigurator().apply(cfgRotate);
            statusTilt = tiltMotor.getConfigurator().apply(cfgTilt);
            statusShooterL = leftShooterMotor.getConfigurator().apply(cfgShooterMotor);
            statusShooterR = rightShooterMotor.getConfigurator().apply(cfgShooterMotor);
            if (statusRotate.isOK())
                break;
        }
        if (!statusRotate.isOK()) {
            System.out.println("Could not configure device. Error: " + statusRotate.toString());
        }
        if (!statusTilt.isOK()) {
            System.out.println("Could not configure device. Error: " + statusTilt.toString());
        }
        if (!statusShooterL.isOK()) {
            System.out.println("Could not configure device. Error: " + statusShooterL.toString());
        }
        if (!statusShooterR.isOK()) {
            System.out.println("Could not configure device. Error: " + statusShooterR.toString());
        }

        leftShooterMotor.setControl(new Follower(0, true));// TODO Set master ID to right shooter
        // TODO Configuration of gear ratio and set them, add analog sensor for position
        // on motor,

        tiltMotor.setPosition(0);
        rotateMotor.setPosition(0);

        rollerMotor.configFactoryDefault();
        rollerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rollerMotor.setSelectedSensorPosition(0);
        rollerMotor.setSensorPhase(false);
        m_bottomEncoderRotationSupplier = () -> Rotation2d
                .fromRotations(rollerMotor.getSelectedSensorPosition() / 4096.0);

        var goalPosition = DriverStation.getAlliance().get().equals(Alliance.Blue)
                ? new Translation2d(Units.inchesToMeters(9), Units.inchesToMeters(218.64))
                : new Translation2d(Units.feetToMeters(54.75) - Units.inchesToMeters(9),
                        Units.inchesToMeters(218.64));

        ShootingUtils.configureShootWhileMoving(() -> goalPosition, chassisSpeedsSupplier,
                () -> swerveDriveStateSupplier.get().Pose.getTranslation(),
                (vector) -> timeOfFlightEquation.get(vector.getNorm()));
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

        public int loopsWithoutNote;

        public Translation2d virtualGoalLocationDisplacement;

        public Rotation2d rotateClosedLoop;

        public Rotation2d tiltClosedLoop;

        public double shooterMotorClosedLoop;

        public boolean activelyIndexingFromIntake;

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

                    rotateMotor.setPosition(m_bottomEncoderRotationSupplier.get().getRotations());

                    m_cachedState.azimuth = Rotation2d.fromRotations(rotateMotor.getPosition().getValueAsDouble());

                    m_cachedState.elevation = Rotation2d.fromRotations(tiltMotor.getPosition().getValueAsDouble());

                    m_cachedState.noteLoaded = sensor.get() || m_cachedState.noteLoaded;

                    m_cachedState.rotateClosedLoop = Rotation2d
                            .fromRotations(rotateMotor.getClosedLoopError().getValueAsDouble());

                    m_cachedState.tiltClosedLoop = Rotation2d
                            .fromRotations(tiltMotor.getClosedLoopError().getValueAsDouble());

                    m_cachedState.shooterMotorClosedLoop = rightShooterMotor.getClosedLoopError().getValueAsDouble();

                    if (sensor.get())
                        m_cachedState.loopsWithoutNote = 0;
                    else
                        m_cachedState.loopsWithoutNote++;
                    if (m_cachedState.loopsWithoutNote >= 5)
                        m_cachedState.noteLoaded = false;

                    try {
                        m_cachedState.virtualGoalLocationDisplacement = ShootingUtils
                                .findVirtualGoalDisplacementFromRobot(0.005, 20, 5);
                    } catch (Exception e) {
                        e.printStackTrace();
                        m_cachedState.virtualGoalLocationDisplacement = null;
                    }

                    m_cachedState.activelyIndexingFromIntake = false;

                    m_requestParameters.turretState = m_cachedState;

                    m_requestToApply.apply(m_requestParameters, rotateMotor, tiltMotor, rollerMotor);
                    m_requestToApplyToShooter.apply(m_requestParameters, rightShooterMotor);
                } finally {
                    m_stateLock.writeLock().unlock();
                }
            }
        }
    }

    public TurretState getState() {
        try {
            m_stateLock.readLock().lock();

            return m_cachedState;
        } finally {
            m_stateLock.readLock().unlock();
        }
    }
}
