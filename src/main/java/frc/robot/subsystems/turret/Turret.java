package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretRequest.TurretControlRequestParameters;
import frc.robot.utils.ShootingUtils;
import frc.robot.utils.sensors.ProximitySensorInput;
import frc.robot.utils.sensors.SensorMonitor;
import frc.robot.utils.shooting.QuadraticEquation;

public class Turret extends SubsystemBase {
    final QuadraticEquation timeOfFlightEquation = new QuadraticEquation().withA(0).withB(0).withC(0);

    TalonFX rightShooterMotor, leftShooterMotor, rotateMotor, tiltMotor;
    TalonSRX rollerMotor;
    ProximitySensorInput sensor;
    Pigeon2 pigeon;

    private final TurretTelemetry turretTelemetry = new TurretTelemetry();

    private final Runnable updateTurretPosition;

    final static double delayNoteLoadedSeconds = 0.05;
    final static double delayNoteUnloadedSeconds = 0.1;

    protected TurretRequest m_requestToApply = new TurretRequest.Idle();
    protected ShooterRequest m_requestToApplyToShooter = new ShooterRequest.Idle();
    protected TurretControlRequestParameters m_requestParameters = new TurretControlRequestParameters();

    private static final boolean enabled = true;
    private static Turret INSTANCE;

    public static Optional<Turret> createInstance() {
        if (INSTANCE != null) {
            return Optional.of(INSTANCE);
        }
        if (!enabled)
            return Optional.empty();
        INSTANCE = new Turret();
        return Optional.of(INSTANCE);
    }

    public static Optional<Turret> getInstance() {
        return Optional.ofNullable(INSTANCE);
    }

    private Turret() {
        super();
        /*
         * Device instantiation
         */
        rightShooterMotor = new TalonFX(17);
        leftShooterMotor = new TalonFX(16);
        rotateMotor = new TalonFX(14);
        tiltMotor = new TalonFX(15);
        rollerMotor = new TalonSRX(18);
        sensor = new ProximitySensorInput(4);
        pigeon = new Pigeon2(22);

        /*
         * Function for configuring motor
         */
        BiConsumer<TalonFX, TalonFXConfiguration> configureMotor = (talon, config) -> {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; i++) {
                status = talon.getConfigurator().apply(config);
                if (status.isOK())
                    break;
            }
            if (!status.isOK()) {
                System.out.println("Could not configure device. Error: " + status.toString());
            }
        };

        /*
         * Apply configurations to motor using above configureMotor BiConsumer
         */
        var configs = Constants.ROBOT.getTurretTalonConfigLiterals();
        configureMotor.accept(rotateMotor, configs.getRotateConfig());
        configureMotor.accept(tiltMotor, configs.getTiltConfig());
        var shooterConfig = configs.getShooterConfig();
        configureMotor.accept(leftShooterMotor, shooterConfig);
        configureMotor.accept(rightShooterMotor, shooterConfig);

        var pigeonConfig = new Pigeon2Configuration();

        // pigeonConfig.MountPose.MountPoseRoll = 180;

        pigeon.getConfigurator().apply(pigeonConfig);

        rightShooterMotor.setInverted(true);
        leftShooterMotor.setInverted(false);

        rotateMotor.setInverted(true);
        rotateMotor.setPosition(0);

        tiltMotor.setInverted(true);
        tiltMotor.setPosition(0);

        rollerMotor.configFactoryDefault();

        var offsetGoalFromWallInches = 0.0;

        var blueGoal = new Translation2d(Units.inchesToMeters(offsetGoalFromWallInches), Units.inchesToMeters(218.64));
        var redGoal = new Translation2d(Units.feetToMeters(54.75) - Units.inchesToMeters(offsetGoalFromWallInches),
                Units.inchesToMeters(218.64));

        ShootingUtils
                .configureShootWhileMoving(
                        () -> DriverStation.getAlliance()
                                .map(alliance -> alliance.equals(Alliance.Blue) ? blueGoal : redGoal).orElse(blueGoal),
                        () -> CommandSwerveDrivetrain.getInstance()
                                .map(drivetrain -> {
                                    var speeds = drivetrain.getFieldRelativeChassisSpeeds();
                                    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                                }).orElse(new Translation2d()),
                        () -> RobotState.getPose()
                                .map((drivetrain) -> drivetrain.getTranslation()).orElse(new Translation2d()),
                        (vector) -> timeOfFlightEquation.get(vector.getNorm()));

        var robotRotationSinceBoot = CommandSwerveDrivetrain.getInstance().map(drivetrain -> {
            m_cachedState.initialRobotRotation = drivetrain.getPigeon2().getRotation2d();
            Supplier<Optional<Rotation2d>> function = () -> Optional
                    .of(Rotation2d.fromRotations(drivetrain.getPigeon2().getRotation2d().getRotations()
                            - m_cachedState.initialRobotRotation.getRotations()));
            return function;
        }).orElse(() -> Optional.empty());

        m_cachedState.initialTurretRotation = pigeon.getRotation2d();
        Supplier<Rotation2d> turretRotationsSinceBoot = () -> Rotation2d
                .fromRotations(
                        pigeon.getRotation2d().getRotations() - m_cachedState.initialTurretRotation.getRotations());

        updateTurretPosition = () -> {
            robotRotationSinceBoot.get().ifPresent(robotRotation -> {
                m_cachedState.robotRotationSinceBoot = robotRotation;
                m_cachedState.turretRotationSinceBoot = turretRotationsSinceBoot.get();
                var calculatedAzimuthFromPigeon = Rotation2d
                        .fromRotations(turretRotationsSinceBoot.get().getRotations() - robotRotation.getRotations());
                rotateMotor.setPosition(calculatedAzimuthFromPigeon.getRotations());
            });
        };
    }

    static int loopCounter = 0;

    @Override
    public void periodic() {
        if (RobotContainer.getRobotState().isResetingClimber()) {
            // TODO reset the offsets so that the turret is at 90 degrees
            // Only on Rising edge of reseting climber
        }

        if (loopCounter++ > 10) {
            updateTurretPosition();

            loopCounter = 0;
        }

        m_cachedState.azimuth = Rotation2d
                .fromRotations(rotateMotor.getPosition().getValueAsDouble());

        m_cachedState.elevation = Rotation2d.fromRotations(tiltMotor.getPosition().getValueAsDouble());

        var rollerOn = Math.abs(rollerMotor.getMotorOutputPercent()) > 0.01;

        if (!m_cachedState.noteLoaded && sensor.get() && rollerOn && !m_cachedState.currentlyShooting) {
            m_cachedState.setNoteLoaded();
        }

        m_cachedState.rollerSupplyCurrent = rollerMotor.getSupplyCurrent();

        m_cachedState.rollerCurrentMonitor.addSensorValue(m_cachedState.rollerSupplyCurrent);

        if (!m_cachedState.rollerCurrentMonitor.hasSignificantMovement()
                && m_cachedState.rollerSupplyCurrent > 10
                && m_cachedState.rollerSupplyCurrent < 18
                && m_cachedState.currentlyShooting) {
            m_cachedState.resetNoteLoaded();
        }

        m_cachedState.rotateClosedLoopError = Rotation2d
                .fromRotations(rotateMotor.getClosedLoopReference().getValueAsDouble()
                        - rotateMotor.getPosition().getValueAsDouble());

        m_cachedState.tiltClosedLoopError = Rotation2d
                .fromRotations(tiltMotor.getClosedLoopReference().getValueAsDouble()
                        - tiltMotor.getPosition().getValueAsDouble());

        m_cachedState.shooterMotorClosedLoopError = rightShooterMotor.getClosedLoopError()
                .getValueAsDouble();

        m_cachedState.virtualGoalLocationDisplacement = ShootingUtils
                .findVirtualGoalDisplacementFromRobot(0.005, 20, 2);

        if (!rollerOn) {
            RobotContainer.getRobotState().setActivelyGrabbing(false);
        }

        RobotContainer.getRobotState().setNoteLoaded(m_cachedState.isNoteLoaded());

        m_cachedState.rotateMonitor.addSensorValue(m_cachedState.azimuth.getDegrees());
        m_cachedState.tiltMonitor.addSensorValue(m_cachedState.elevation.getDegrees());

        m_cachedState.seeingNote = sensor.get();

        m_requestParameters.turretState = m_cachedState;

        turretTelemetry.telemetrize(m_cachedState);
    }

    public Command applyRequest(Supplier<TurretRequest> requestSupplier,
            Supplier<ShooterRequest> shooterRequestSupplier) {
        return run(() -> this.setControl(requestSupplier.get(), shooterRequestSupplier.get()));
    }

    private void setControl(TurretRequest request, ShooterRequest shooterRequest) {
        m_requestToApply = request;
        m_requestToApplyToShooter = shooterRequest;
        m_requestToApply.apply(m_requestParameters, rotateMotor, tiltMotor, rollerMotor);
        m_requestToApplyToShooter.apply(m_requestParameters, rightShooterMotor, leftShooterMotor);
    }

    public class TurretState {

        public Rotation2d initialRobotRotation, initialTurretRotation;

        Rotation2d azimuth, elevation;

        double rollerSupplyCurrent;

        boolean seeingNote;

        boolean noteLoaded;

        Rotation2d robotRotationSinceBoot, turretRotationSinceBoot;

        final Timer noteLoadedTimer;

        Optional<Translation2d> virtualGoalLocationDisplacement;

        Rotation2d rotateClosedLoopError, tiltClosedLoopError;

        double shooterMotorClosedLoopError;

        boolean currentlyShooting;

        boolean unloadedTimerStarted = false;

        protected double currentRotateAccel = 0;

        double delayNoteLoadedSeconds = Turret.delayNoteLoadedSeconds;

        SensorMonitor rotateMonitor = new SensorMonitor(1.0, 0.020, 10.0); // check for 1 second, max difference should
                                                                           // not exceed 5 degrees within that time
                                                                           // period

        SensorMonitor tiltMonitor = new SensorMonitor(1.0, 0.020, 10.0);

        SensorMonitor rollerCurrentMonitor = new SensorMonitor(Turret.delayNoteUnloadedSeconds, 0.020, 3);

        TurretState() {
            noteLoadedTimer = new Timer();
            robotRotationSinceBoot = new Rotation2d();
            turretRotationSinceBoot = new Rotation2d();
        }

        public boolean isNoteLoaded() {
            return noteLoaded && noteLoadedTimer.hasElapsed(delayNoteLoadedSeconds);
        }

        public void setNoteLoaded() {
            this.noteLoaded = true;
            this.noteLoadedTimer.restart();
            delayNoteLoadedSeconds = Turret.delayNoteLoadedSeconds;
        }

        public void setNoteLoadedNoDelay() {
            setNoteLoaded();
            delayNoteLoadedSeconds = 0;
        }

        public void resetNoteLoaded() {
            noteLoaded = false;
        }

        public Optional<Translation2d> getVirtualGoalLocationDisplacement() {
            return virtualGoalLocationDisplacement;
        }

        public Rotation2d getCurrentAzimuth() {
            return this.azimuth;
        }

        public boolean hasTurretAnglesBeenStill() {
            return !(rotateMonitor.hasSignificantMovement() || tiltMonitor.hasSignificantMovement());
        }

    }

    final TurretState m_cachedState = new TurretState();

    public void resetNoteLoaded() {
        m_cachedState.resetNoteLoaded();
    }

    public void setNoteLoaded() {
        m_cachedState.setNoteLoaded();
    }

    public void setNoteLoadedNoDelay() {
        m_cachedState.setNoteLoadedNoDelay();
    }

    public void updateTurretPosition() {
        updateTurretPosition.run();
    }

}
