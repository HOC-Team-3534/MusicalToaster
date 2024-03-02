package frc.robot.subsystems.swervedrive;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swervedrive.path.IPathPlanner.PathPlanner2024;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    FeedForwardCharacterizationData characterizationData = new FeedForwardCharacterizationData();
    double timeCharacterizing;
    double maxSpeed;
    PathPlanner2024 pathplanner = new PathPlanner2024();
    // private final Telemetry logger;

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

        // logger = new Telemetry(maxSpeed);

        // registerTelemetry(logger::telemeterize);
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
        var request = new CharacterizationVoltage().withMaxSpeed(maxSpeed);
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

class FeedForwardCharacterizationData {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public void add(double velocity, double voltage) {
        if (Math.abs(velocity) > 1E-4) {
            velocityData.add(Math.abs(velocity));
            voltageData.add(Math.abs(voltage));
        }
    }

    public void print() {
        double velocityDataArray[] = velocityData.stream().mapToDouble(Double::doubleValue).toArray();
        double voltageDataArray[] = voltageData.stream().mapToDouble(Double::doubleValue).toArray();
        double accelerationDataArray[] = new double[velocityDataArray.length];
        for (int i = 0; i < velocityDataArray.length - 1; i++) {
            accelerationDataArray[i] = (velocityDataArray[i + 1] - velocityDataArray[i]) / 0.020;
        }
        accelerationDataArray[accelerationDataArray.length - 1] = accelerationDataArray[accelerationDataArray.length
                - 2];
        PolynomialRegression regression = new PolynomialRegression(velocityDataArray,
                voltageDataArray, 1);
        double residualsVoltageVelocityWise[] = new double[velocityDataArray.length];
        for (int i = 0; i < velocityDataArray.length; i++) {
            residualsVoltageVelocityWise[i] = voltageDataArray[i] - regression.predict(velocityDataArray[i]);
        }
        PolynomialRegression accelerationRegression = new PolynomialRegression(accelerationDataArray,
                residualsVoltageVelocityWise,
                1);
        System.out.println("FF Characterization Results:");
        System.out.println("\tCount=" + Integer.toString(velocityData.size())
                + "");
        System.out.println(String.format("\tR2=%.5f", regression.R2(velocityDataArray, voltageDataArray)));
        System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
        System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
        System.out.println(String.format("\tkA=%.5f", accelerationRegression.beta(1)));
    }
}

class PolynomialRegression {
    private SimpleMatrix coefficients;

    public PolynomialRegression(double[] x, double[] y, int degree) {
        // Add a column of ones to the x matrix for the y-intercept
        SimpleMatrix xMatrix = new SimpleMatrix(x.length, degree + 1);
        for (int i = 0; i < x.length; i++) {
            xMatrix.set(i, 0, 1);
            for (int j = 1; j <= degree; j++) {
                xMatrix.set(i, j, Math.pow(x[i], j));
            }
        }
        // Use the Moore-Penrose pseudoinverse to find the coefficients
        SimpleMatrix yMatrix = new SimpleMatrix(y.length, 1, true, y);
        SimpleMatrix transpose = xMatrix.transpose();
        SimpleMatrix xTx = transpose.mult(xMatrix);
        SimpleMatrix xTxInv = xTx.invert();
        SimpleMatrix xTy = transpose.mult(yMatrix);
        coefficients = xTxInv.mult(xTy);
    }

    public double predict(double x) {
        double y = 0;
        for (int i = 0; i < coefficients.getNumRows(); i++) {
            y += coefficients.get(i) * Math.pow(x, i);
        }
        return y;
    }

    public double R2(double[] x, double[] y) {

        double sst = 0, sse = 0, yMean = 0;
        for (int i = 0; i < y.length; i++) {
            yMean += y[i];
        }
        yMean /= y.length;
        for (int i = 0; i < y.length; i++) {
            double yPred = predict(x[i]);
            sst += (y[i] - yMean) * (y[i] - yMean);
            sse += (y[i] - yPred) * (y[i] - yPred);
        }
        return 1 - sse / sst;
    }

    public double beta(int degree) {
        return coefficients.get(degree);
    }
}

class CharacterizationVoltage implements SwerveRequest {

    /**
     * The voltage to apply to the drive motors.
     */
    public double VoltageX = 0;

    /**
     * The maxVelocity of the robot in m/s
     */
    public double MaxSpeed = 0;

    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        SwerveModuleState state = new SwerveModuleState(VoltageX / 12.0 * MaxSpeed, new Rotation2d());

        for (int i = 0; i < modulesToApply.length; ++i) {
            modulesToApply[i].apply(state, DriveRequestType, SteerRequestType);
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

    /**
     * Sets the max speed of the robot
     *
     * @param maxSpeed
     *                 max speed of the robot in m/s
     * @return this request
     */
    public CharacterizationVoltage withMaxSpeed(double maxSpeed) {
        this.MaxSpeed = maxSpeed;
        return this;
    }

}