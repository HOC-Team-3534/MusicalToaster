package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretRequest.TurretControlRequestParameters;
import frc.robot.utils.ShootingUtils;
import frc.robot.utils.sensors.ProximitySensorInput;
import frc.robot.utils.shooting.QuadraticEquation;

public class Turret extends SubsystemBase {
    final QuadraticEquation timeOfFlightEquation = new QuadraticEquation().withA(0).withB(0).withC(0);

    TalonFX rightShooterMotor, leftShooterMotor, rotateMotor, tiltMotor;
    TalonSRX rollerMotor;
    ProximitySensorInput sensor;

    private final TurretTelemetry turretTelemetry = new TurretTelemetry();

    final static double delayNoteLoadedSeconds = 0.1;
    final static double delayNoteUnloadedSeconds = 1.0;

    protected TurretRequest m_requestToApply = new TurretRequest.Idle();
    protected ShooterRequest m_requestToApplyToShooter = new ShooterRequest.Idle();
    protected TurretControlRequestParameters m_requestParameters = new TurretControlRequestParameters();

    private static final boolean enabled = true;
    private static Turret INSTANCE;

    private Supplier<Boolean> hardSetNoteInRobotSupplier;

    public static Optional<Turret> createInstance(Supplier<Boolean> hardSetNoteInRobotSupplier) {
        if (INSTANCE != null) {
            return Optional.of(INSTANCE);
        }
        if (!enabled)
            return Optional.ofNullable(null);
        INSTANCE = new Turret(hardSetNoteInRobotSupplier);
        return Optional.of(INSTANCE);
    }

    public static Optional<Turret> getInstance() {
        return Optional.ofNullable(INSTANCE);
    }

    private Turret(
            Supplier<Boolean> hardSetNoteInRobotSupplier) {
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

        /*
         * Rotate Motor Configuration
         */
        TalonFXConfiguration cfgRotate = new TalonFXConfiguration();

        cfgRotate.MotionMagic.MotionMagicCruiseVelocity = 0.75;
        cfgRotate.MotionMagic.MotionMagicAcceleration = 0.15;
        cfgRotate.MotionMagic.MotionMagicJerk = 20;

        cfgRotate.Slot0.kP = 120;
        cfgRotate.Slot0.kV = 13.29;// TODO Tune these values
        cfgRotate.Slot0.kS = 0.1763;
        cfgRotate.Slot0.kD = 2.0;

        cfgRotate.Feedback.SensorToMechanismRatio = 125;

        /*
         * Tilt Motor Configuration
         */
        TalonFXConfiguration cfgTilt = new TalonFXConfiguration();

        cfgTilt.MotionMagic.MotionMagicCruiseVelocity = 0.85;
        cfgTilt.MotionMagic.MotionMagicAcceleration = 1.0;
        cfgTilt.MotionMagic.MotionMagicJerk = 10;

        cfgTilt.Slot0.kP = 10;
        cfgTilt.Slot0.kV = 32.55;// TODO Tune these values
        cfgTilt.Slot0.kS = 0.13;

        cfgTilt.Feedback.SensorToMechanismRatio = 300;

        /*
         * Shooter Motor Configuration
         */

        TalonFXConfiguration cfgShooter = new TalonFXConfiguration();

        cfgShooter.Slot0.kP = 5;
        cfgShooter.Slot0.kI = 0;
        cfgShooter.Slot0.kA = 0.0115;
        cfgShooter.Slot0.kV = 0.0230234;
        cfgShooter.Slot0.kS = 1.5289;

        cfgShooter.Feedback.SensorToMechanismRatio = 1;

        cfgShooter.CurrentLimits.SupplyCurrentLimit = 30;

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
        configureMotor.accept(rotateMotor, cfgRotate);
        configureMotor.accept(tiltMotor, cfgTilt);
        configureMotor.accept(leftShooterMotor, cfgShooter);
        configureMotor.accept(rightShooterMotor, cfgShooter);

        rightShooterMotor.setInverted(true);
        leftShooterMotor.setInverted(false);

        rotateMotor.setInverted(true);
        rotateMotor.setPosition(0);

        tiltMotor.setInverted(true);
        tiltMotor.setPosition(0);

        rollerMotor.configFactoryDefault();
        rollerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rollerMotor.setSelectedSensorPosition(0);
        rollerMotor.setSensorPhase(true);

        var blueGoal = new Translation2d(Units.inchesToMeters(9), Units.inchesToMeters(218.64));
        var redGoal = new Translation2d(Units.feetToMeters(54.75) - Units.inchesToMeters(9),
                Units.inchesToMeters(218.64));

        ShootingUtils.configureShootWhileMoving(() -> {
            if (DriverStation.getAlliance().isEmpty())
                return blueGoal;
            return DriverStation.getAlliance().get().equals(Alliance.Blue)
                    ? blueGoal
                    : redGoal;
        }, () -> CommandSwerveDrivetrain.getInstance().map((drivetrain) -> drivetrain.getChassisSpeeds())
                .orElse(new ChassisSpeeds()),
                () -> RobotContainer.getPose().map((drivetrain) -> drivetrain.getTranslation())
                        .orElse(new Translation2d()),
                (vector) -> timeOfFlightEquation.get(vector.getNorm()));
        this.hardSetNoteInRobotSupplier = hardSetNoteInRobotSupplier;
    }

    @Override
    public void periodic() {
        m_cachedState.rawAzimuthEncoderCounts = rollerMotor.getSelectedSensorPosition();

        m_cachedState.azimuth = Rotation2d.fromRotations(m_cachedState.rawAzimuthEncoderCounts / 1440.0);

        rotateMotor.setPosition(m_cachedState.azimuth.getRotations());

        m_cachedState.azimuthFromMotor = Rotation2d
                .fromRotations(rotateMotor.getPosition().getValueAsDouble());

        m_cachedState.rawElevationRotations = tiltMotor.getPosition().getValueAsDouble();

        m_cachedState.elevation = Rotation2d.fromRotations(tiltMotor.getPosition().getValueAsDouble());

        var rollerOn = Math.abs(rollerMotor.getMotorOutputPercent()) > 0.01;

        if (!m_cachedState.noteLoaded && sensor.get() && rollerOn && !m_cachedState.currentlyShooting) {
            m_cachedState.noteLoaded = true;
            m_cachedState.noteLoadedTimer.restart();
            m_cachedState.noteUnloadedTimer.stop();
        }

        if (m_cachedState.currentlyShooting && rollerOn) {
            m_cachedState.noteUnloadedTimer.restart();
        }

        if (m_cachedState.noteUnloadedTimer.hasElapsed(delayNoteUnloadedSeconds)) {
            m_cachedState.noteLoaded = false;
            m_cachedState.noteUnloadedTimer.reset();
        }

        m_cachedState.rotateClosedLoopError = Rotation2d
                .fromRotations(rotateMotor.getClosedLoopReference().getValueAsDouble()
                        - rotateMotor.getPosition().getValueAsDouble());

        m_cachedState.tiltClosedLoopError = Rotation2d
                .fromRotations(tiltMotor.getClosedLoopReference().getValueAsDouble()
                        - tiltMotor.getPosition().getValueAsDouble());

        m_cachedState.shooterMotorClosedLoopError = rightShooterMotor.getClosedLoopError()
                .getValueAsDouble();

        try {
            m_cachedState.virtualGoalLocationDisplacement = ShootingUtils
                    .findVirtualGoalDisplacementFromRobot(0.005, 20, 5);
        } catch (Exception e) {
            e.printStackTrace();
            m_cachedState.virtualGoalLocationDisplacement = null;
        }

        if (!rollerOn) {
            RobotContainer.setActivelyGrabbing(false);
        }

        RobotContainer.setNoteLoaded(m_cachedState.isNoteLoaded());

        m_cachedState.currentlyShooting = false;
        if (hardSetNoteInRobotSupplier.get()) {
            m_cachedState.noteLoaded = true;
        }
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
        public Rotation2d azimuth;

        public double rawAzimuthEncoderCounts;

        public Rotation2d azimuthFromMotor;

        public Rotation2d elevation;

        public double rawElevationRotations;

        private boolean noteLoaded;

        private Timer noteLoadedTimer = new Timer();

        private Timer noteUnloadedTimer = new Timer();

        public Translation2d virtualGoalLocationDisplacement;

        public Rotation2d rotateClosedLoopError;

        public Rotation2d tiltClosedLoopError;

        public double shooterMotorClosedLoopError;

        public boolean currentlyShooting;

        public boolean isNoteLoaded() {
            return noteLoaded && noteLoadedTimer.hasElapsed(delayNoteLoadedSeconds);
        }

    }

    final TurretState m_cachedState = new TurretState();

}
