package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretRequest.TurretControlRequestParameters;
import frc.robot.utils.ShootingUtils;
import frc.robot.utils.sensors.ProximitySensorInput;
import frc.robot.utils.sensors.QuadEncoderOnSRX;
import frc.robot.utils.sensors.SensorMonitor;
import frc.robot.utils.shooting.QuadraticEquation;

public class Turret extends SubsystemBase {
    final QuadraticEquation timeOfFlightEquation = new QuadraticEquation().withA(0).withB(0).withC(0);

    final TalonFX rightShooterMotor, leftShooterMotor, rotateMotor, tiltMotor;
    final TalonSRX rollerMotor;
    final ProximitySensorInput sensor;
    final QuadEncoderOnSRX quadEncoder;
    final Pigeon2 pigeon2;

    private final TurretTelemetry turretTelemetry = new TurretTelemetry();

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
        pigeon2 = new Pigeon2(22);

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

        pigeon2.getConfigurator().apply(pigeonConfig);

        rightShooterMotor.setInverted(true);
        leftShooterMotor.setInverted(false);

        rotateMotor.setInverted(true);
        rotateMotor.setPosition(0);

        tiltMotor.setInverted(true);
        tiltMotor.setPosition(0);

        rollerMotor.configFactoryDefault();

        quadEncoder = new QuadEncoderOnSRX(rollerMotor).withSensorPhase(true);
        quadEncoder.setRotation(new Rotation2d());

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
    }

    static int loopCounter = 0;

    @Override
    public void periodic() {
        if (RobotContainer.getRobotState().isResetingClimber()) {
            quadEncoder.setRotation(Rotation2d.fromDegrees(90));
        }

        m_cachedState.azimuth = quadEncoder.getRotation();

        m_cachedState.elevation = Rotation2d.fromDegrees(pigeon2.getPitch().getValueAsDouble());

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
        m_requestToApply.apply(m_requestParameters, this::controlRotate, this::controlTilt, rollerMotor);
        m_requestToApplyToShooter.apply(m_requestParameters, rightShooterMotor, leftShooterMotor);
    }

    static double rotateHowQuickToResolveError = 0.1;
    static double tiltHowQuickToResolveError = 0.55; // DO NOT GO BELOW 0.55 seconds

    ProfiledPIDController rotatePID = new ProfiledPIDController(1.0 / rotateHowQuickToResolveError, 0, 0,
            new TrapezoidProfile.Constraints(0.3, 0.25));
    ProfiledPIDController tiltPID = new ProfiledPIDController(1.0 / tiltHowQuickToResolveError, 0, 0,
            new TrapezoidProfile.Constraints(0.00, 0.00));

    SimpleMotorFeedforward rotateFeedforward = new SimpleMotorFeedforward(0.162, 14.6);
    SimpleMotorFeedforward tiltFeedforward = new SimpleMotorFeedforward(0.216898, 32.0713);

    VoltageOut rotateOut = new VoltageOut(0);
    VoltageOut tiltOut = new VoltageOut(0);

    final static double MAX_BACKLASH_VELOCITY_ROTATE = 0.05;
    final static double MAX_BACKLASH_VELOCITY_TILT = 0.015;

    private void controlRotate(Rotation2d targetRotation) {
        rotateOut.withOutput(0);

        var rotateError = 0.0;

        if (targetRotation != null) {
            m_cachedState.rotateBacklashSensorMonitor.addSensorValue(m_cachedState.azimuth.getDegrees());

            m_cachedState.rotateVelocityOut = rotatePID.calculate(m_cachedState.azimuth.getRotations(),
                    targetRotation.getRotations());

            if (!m_cachedState.rotateBacklashSensorMonitor.hasSignificantMovement()
                    && Math.abs(m_cachedState.rotateVelocityOut) > MAX_BACKLASH_VELOCITY_ROTATE) {
                m_cachedState.rotateVelocityOut = Math.copySign(MAX_BACKLASH_VELOCITY_ROTATE,
                        m_cachedState.rotateVelocityOut);
            }

            var voltageOutput = rotateFeedforward.calculate(m_cachedState.rotateVelocityOut);

            rotateOut.withOutput(voltageOutput);

            rotateError = rotatePID.getPositionError();
        }

        rotateMotor.setControl(rotateOut);

        m_cachedState.rotateClosedLoopError = Rotation2d.fromRotations(rotateError);
    }

    private void controlTilt(Rotation2d targetTilt) {
        tiltOut.withOutput(0);

        var tiltError = 0.0;

        if (targetTilt != null) {
            m_cachedState.tiltBacklashSensorMonitor.addSensorValue(m_cachedState.elevation.getDegrees());

            m_cachedState.tiltVelocityOut = tiltPID.calculate(m_cachedState.elevation.getRotations(),
                    targetTilt.getRotations());

            if (!m_cachedState.tiltBacklashSensorMonitor.hasSignificantMovement()
                    && Math.abs(m_cachedState.tiltVelocityOut) > MAX_BACKLASH_VELOCITY_TILT) {
                m_cachedState.tiltVelocityOut = Math.copySign(MAX_BACKLASH_VELOCITY_TILT,
                        m_cachedState.tiltVelocityOut);
            }

            var voltageOutput = tiltFeedforward.calculate(m_cachedState.tiltVelocityOut);

            tiltOut.withOutput(voltageOutput);

            m_cachedState.tiltSetpointPosition = tiltPID.getSetpoint().position;

            tiltError = tiltPID.getPositionError();
        }

        tiltMotor.setControl(tiltOut);

        m_cachedState.tiltClosedLoopError = Rotation2d.fromRotations(tiltError);
    }

    public class TurretState {

        double tiltSetpointPosition;

        Rotation2d azimuth, elevation;

        double rollerSupplyCurrent;

        boolean seeingNote;

        boolean noteLoaded;

        final Timer noteLoadedTimer;

        double rotateVelocityOut, tiltVelocityOut;

        Optional<Translation2d> virtualGoalLocationDisplacement;

        Rotation2d rotateClosedLoopError, tiltClosedLoopError;

        double shooterMotorClosedLoopError;

        boolean currentlyShooting;

        boolean unloadedTimerStarted = false;

        SensorMonitor rotateBacklashSensorMonitor = new SensorMonitor(0.25, 0.020, 1.0);
        SensorMonitor tiltBacklashSensorMonitor = new SensorMonitor(0.25, 0.020, 0.5);

        protected double currentRotateAccel = 0;

        double delayNoteLoadedSeconds = Turret.delayNoteLoadedSeconds;

        SensorMonitor rotateMonitor = new SensorMonitor(2.0, 0.020, 2.0); // check for 1 second, max difference should
                                                                          // not exceed 5 degrees within that time
                                                                          // period

        SensorMonitor tiltMonitor = new SensorMonitor(1.0, 0.020, 2.0);

        SensorMonitor rollerCurrentMonitor = new SensorMonitor(Turret.delayNoteUnloadedSeconds, 0.020, 3);

        TurretState() {
            noteLoadedTimer = new Timer();
            rotateClosedLoopError = new Rotation2d();
            tiltClosedLoopError = new Rotation2d();
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

}
