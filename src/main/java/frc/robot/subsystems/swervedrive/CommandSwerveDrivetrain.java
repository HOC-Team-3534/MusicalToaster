package frc.robot.subsystems.swervedrive;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.path.IPathPlanner.PathPlanner2024;
import frc.robot.utils.characterization.FeedForwardCharacterizationData;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    FeedForwardCharacterizationData characterizationData = new FeedForwardCharacterizationData();
    double timeCharacterizing;
    double maxSpeed;
    PathPlanner2024 pathplanner = new PathPlanner2024();
    private final Telemetry logger;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            double maxSpeed, HolonomicPathFollowerConfig holoConfig,
            SwerveModuleConstants... modules) {
        this(driveTrainConstants, OdometryUpdateFrequency,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9), maxSpeed, holoConfig, modules);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            double maxSpeed, HolonomicPathFollowerConfig holoConfig,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        this.maxSpeed = maxSpeed;
        pathplanner.configureHolonomic(() -> this.getState().Pose, this::seedFieldRelative,
                this::getChassisSpeeds,
                speeds -> this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)), holoConfig,
                () -> false, this);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        logger = new Telemetry(maxSpeed);

        registerTelemetry(logger::telemeterize);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double maxSpeed,
            HolonomicPathFollowerConfig holoConfig, SwerveModuleConstants... modules) {
        this(driveTrainConstants, 0, maxSpeed, holoConfig, modules);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command pathfindToPose(Pose2d endPose, PathConstraints pathConstraints, double endVelocity) {
        return pathplanner.pathfindToPose(endPose, pathConstraints, endVelocity);
    }

    public Command pathfindToPose(Translation2d endPosition, PathConstraints pathConstraints, double endVelocity) {
        return pathplanner.pathfindToPose(endPosition, this::faceAnySideOfRobotInDirectionOfTravel, pathConstraints,
                endVelocity);
    }

    public Rotation2d faceAnySideOfRobotInDirectionOfTravel(PathPlannerTrajectory.State state) {
        var currentHolonomicRotation = getState().Pose.getRotation();
        var travelDirection = state.heading;

        var diff = travelDirection.minus(currentHolonomicRotation);

        var diffDegrees = diff.getDegrees() % 360; // -360 to 360
        while (diffDegrees < -45) {
            diffDegrees += 90;
        }
        while (diffDegrees > 45) {
            diffDegrees -= 90;
        }
        // diffDegrees between -45 to 45;

        return Rotation2d.fromDegrees(currentHolonomicRotation.getDegrees() % 360 + diffDegrees);
    }

    public Rotation2d faceFrontTowardsRobotDirectionOfTravel(PathPlannerTrajectory.State state) {
        var travelDirection = state.heading;

        return Rotation2d.fromDegrees(travelDirection.getDegrees() % 360);
    }

    public Command pathfindToPose(Translation2d endPosition,
            Function<PathPlannerTrajectory.State, Rotation2d> rotationFunction, PathConstraints pathConstraints,
            double endVelocity) {
        return pathplanner.pathfindToPose(endPosition, rotationFunction, pathConstraints, endVelocity);
    }

    public Command followPath(PathPlannerPath path) {
        return pathplanner.followPath(path);
    }

    /**
     * 
     * @param quas_voltage
     *                      the quasiastic voltage per second
     * @param quas_duration
     *                      the quasiastic test duration
     * @return the command to characterize the swerve drivetrain
     */
    public Command characterizeDrive(double quas_voltage, double quas_duration) {
        var request = new CharacterizationVoltage();
        return runOnce(() -> {
            this.setControl(request.withVoltageX(0));
            characterizationData = new FeedForwardCharacterizationData();
            timeCharacterizing = 0;
        }).andThen(
                runEnd(() -> {
                    timeCharacterizing += 0.020;
                    this.setControl(request.withVoltageX(timeCharacterizing * quas_voltage));
                    var fl = this.getModule(0);
                    var voltage = fl.getDriveMotor().getMotorVoltage().getValueAsDouble();
                    var velocityMPS = fl.getCurrentState().speedMetersPerSecond;
                    characterizationData.add(velocityMPS, voltage);
                }, () -> {
                    this.setControl(new SwerveRequest.Idle());
                    characterizationData.print();
                })).until(() -> timeCharacterizing >= quas_duration);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}

class CharacterizationVoltage implements SwerveRequest {

    /**
     * The voltage to apply to the drive motors.
     */
    double VoltageX = 0;

    VoltageOut voltageOut = new VoltageOut(0);

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].applyCharacterization(new Rotation2d(), voltageOut.withOutput(VoltageX));
        }

        return StatusCode.OK;
    }

    /**
     * Sets the voltage of the drive motors.
     *
     * @param voltageX
     *                 Voltage of the drive motors
     * @return this request
     */
    public CharacterizationVoltage withVoltageX(double voltageX) {
        this.VoltageX = voltageX;
        return this;
    }

}