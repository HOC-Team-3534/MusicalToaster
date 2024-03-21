package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretRequest.TurretControlRequestParameters;
import frc.robot.utils.ShootingUtils;
import frc.robot.utils.motioncontrol.statemachine.MotionProfileStateMachine;
import frc.robot.utils.motioncontrol.statemachine.OutputConstraints;
import frc.robot.utils.motioncontrol.statemachine.StateManagement;
import frc.robot.utils.sensors.ProximitySensorInput;
import frc.robot.utils.sensors.QuadEncoderOnSRX;
import frc.robot.utils.sensors.SensorMonitor;
import frc.robot.utils.shooting.QuadraticEquation;

public class Turret extends SubsystemBase {
    final QuadraticEquation timeOfFlightEquation = new QuadraticEquation().withA(0.00983625).withB(0.0397349)
            .withC(0.082);

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

    final SimpleMotorFeedforward tiltFeedforward = new SimpleMotorFeedforward(0.2169, 32.07);
    final SimpleMotorFeedforward rotateFeedforward = new SimpleMotorFeedforward(0.162, 14.6);

    final MotionProfileStateMachine tiltStateMachine, rotateStateMachine;

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

        rotateStateMachine = new MotionProfileStateMachine(() -> m_cachedState.azimuth.getRotations(), 1.0 / 360.0,
                0.1, 10.0 / 360.0, new OutputConstraints(0.7, 1.5, 0.2, 0.035));

        tiltStateMachine = new MotionProfileStateMachine(() -> m_cachedState.elevation.getRotations(), 0.5 / 360.0,
                0.1, 5.0 / 360.0, new OutputConstraints(0.4, 0.5, 0.3, 0.015));

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

    VoltageOut rotateOut = new VoltageOut(0);
    VoltageOut tiltOut = new VoltageOut(0);

    private void controlRotate(Rotation2d targetRotation) {

        double outputVelocity = 0;

        if (targetRotation != null) {

            outputVelocity = rotateStateMachine.determineStateAndCalculateVelocity(targetRotation.getRotations());

            rotateOut.withOutput(rotateFeedforward.calculate(outputVelocity));
        } else {
            rotateOut.withOutput(0);
        }

        m_cachedState.rotateVelocityOut = outputVelocity;
        m_cachedState.rotateClosedLoopError = Rotation2d.fromRotations(rotateStateMachine.getPositionError());
        m_cachedState.rotateVelocityError = Rotation2d.fromRotations(rotateStateMachine.getVelocityError());
        m_cachedState.rotateState = rotateStateMachine.getState().name();
        m_cachedState.rotateStateOrdinal = rotateStateMachine.getState().ordinal();

        rotateMotor.setControl(rotateOut);
    }

    private void controlTilt(Rotation2d targetRotation) {

        double outputVelocity = 0;

        if (targetRotation != null) {

            outputVelocity = tiltStateMachine.determineStateAndCalculateVelocity(targetRotation.getRotations());

            if (Math.abs(targetRotation.getDegrees()) < 2
                    && Math.abs(tiltStateMachine.getPositionError()) < 2.0 / 360.0)
                outputVelocity = 0;

            tiltOut.withOutput(tiltFeedforward.calculate(outputVelocity));
        } else {
            tiltOut.withOutput(0);
        }

        m_cachedState.tiltVelocityOut = outputVelocity;
        m_cachedState.tiltClosedLoopError = Rotation2d.fromRotations(tiltStateMachine.getPositionError());
        m_cachedState.tiltVelocityError = Rotation2d.fromRotations(tiltStateMachine.getVelocityError());
        m_cachedState.tiltState = tiltStateMachine.getState().name();
        m_cachedState.tiltStateOrdinal = tiltStateMachine.getState().ordinal();

        tiltMotor.setControl(tiltOut);
    }

    public class TurretState {

        Rotation2d rotateVelocityError, tiltVelocityError;

        Rotation2d azimuth, elevation;

        String tiltState = "Stationary", rotateState = "Stationary";

        int tiltStateOrdinal = StateManagement.State.Stationary.ordinal(),
                rotateStateOrdinal = StateManagement.State.Stationary.ordinal();

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
            rotateVelocityError = new Rotation2d();
            tiltVelocityError = new Rotation2d();
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
