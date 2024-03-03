package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.Turret.TurretState;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TurretUtils;

public interface TurretRequest {
    public class TurretControlRequestParameters {
        TurretState turretState;
    }

    public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
            TalonSRX rollerMotor);

    static final double lowerLimit_azimuthDegrees = -240;
    static final double upperLimit_azimuthDegrees = 240;

    static final double lowerLimit_elevationDegrees = -50;
    static final double upperLimit_elevationDegrees = 50;

    static final Rotation2d azimuthTolerance = Rotation2d.fromDegrees(3);
    static final Rotation2d elevationTolerance = Rotation2d.fromDegrees(1);
    static final double shooterTolerance = 5.0;
    static final double rollerShootPercentOut = -0.75;

    public class IndexFromIntake implements TurretRequest {
        private Rotation2d tilt;
        private double rollerPercentOut;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            parameters.turretState.currentlyShooting = false;

            int index = RobotContainer.getRobotState().grabNoteIndex;
            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;
            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var targetAzimuth = currentAzimuth;

            if (!parameters.turretState.isNoteLoaded() && index != -1) {
                targetAzimuth = Rotation2d.fromRotations(0.25 * (index + 2));
                targetAzimuth = TurretUtils.calculateTargetAzimuthWithinLimits(targetAzimuth, currentAzimuth,
                        lowerLimit_azimuthDegrees,
                        upperLimit_azimuthDegrees);

                outputTilt = RobotContainer.isTiltForcedFlat()
                        ? new Rotation2d()
                        : tilt;

                var azimuthError = targetAzimuth.minus(currentAzimuth);
                var elevationError = outputTilt.minus(currentElevation);

                if (MathUtils.withinTolerance(azimuthError, azimuthTolerance)
                        && MathUtils.withinTolerance(elevationError, elevationTolerance)) {
                    RobotContainer.setActivelyGrabbing(true);
                    rollerOn = true;
                }
            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(targetAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));
            return StatusCode.OK;
        }

        public IndexFromIntake withTilt(Rotation2d tilt) {
            this.tilt = tilt;
            return this;
        }

        public IndexFromIntake withRollerOutput(double percentOut) {
            this.rollerPercentOut = percentOut;
            return this;
        }
    }

    public class ControlTurret implements TurretRequest {

        // Given parameters and rotation of robot, determine rotation of turret
        Function<TurretControlRequestParameters, Rotation2d> targetAzimuthFunction;
        Function<TurretControlRequestParameters, Rotation2d> targetElevationFunction;
        Supplier<Boolean> allowShootWhenAimedSupplier;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            boolean rollerOn = false;
            Rotation2d targetAzimuth;
            Rotation2d targetElevation;

            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;

            targetAzimuth = currentAzimuth;
            targetElevation = currentElevation;

            if (parameters.turretState.isNoteLoaded()) {
                /*
                 * Azimuth Calculation
                 */
                targetAzimuth = targetAzimuthFunction.apply(parameters);
                targetAzimuth = TurretUtils.calculateTargetAzimuthWithinLimits(targetAzimuth, currentAzimuth,
                        lowerLimit_azimuthDegrees,
                        upperLimit_azimuthDegrees);

                /*
                 * Elevation Calculation
                 */
                targetElevation = RobotContainer.isTiltForcedFlat()
                        ? new Rotation2d()
                        : targetElevationFunction.apply(parameters);
                targetElevation = Rotation2d.fromDegrees(MathUtils.applyUpperAndLowerLimit(targetElevation.getDegrees(),
                        lowerLimit_elevationDegrees, upperLimit_elevationDegrees));

                /*
                 * Error Calculation
                 */

                var azimuthError = targetAzimuth.minus(currentAzimuth);
                var elevationError = targetElevation.minus(currentElevation);
                var shooterError = parameters.turretState.shooterMotorClosedLoopError;

                if ((MathUtils.withinTolerance(azimuthError, azimuthTolerance)
                        && MathUtils.withinTolerance(elevationError, elevationTolerance)
                        && MathUtils.withinTolerance(shooterError, shooterTolerance)
                        && allowShootWhenAimedSupplier.get())
                        || parameters.turretState.currentlyShooting) {
                    rollerOn = true;
                }

            }

            parameters.turretState.currentlyShooting = rollerOn;

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerShootPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(targetAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(targetElevation.getRotations()));

            return StatusCode.OK;
        }

        public ControlTurret withTargetAzimuthFunction(
                Function<TurretControlRequestParameters, Rotation2d> targetAzimuthFunction) {
            this.targetAzimuthFunction = targetAzimuthFunction;
            return this;
        }

        public ControlTurret withTargetElevationFunction(
                Function<TurretControlRequestParameters, Rotation2d> targetElevationFunction) {
            this.targetElevationFunction = targetElevationFunction;
            return this;
        }

        public ControlTurret withAllowShootWhenAimedSupplier(Supplier<Boolean> allowShootWhenAimedSupplier) {
            this.allowShootWhenAimedSupplier = allowShootWhenAimedSupplier;
            return this;
        }

    }

    public class AimForSpeaker implements TurretRequest {
        private Rotation2d tolerance = Rotation2d.fromDegrees(3);
        private Rotation2d tiltTolerance = Rotation2d.fromDegrees(0.5);
        private double shooterTolerance;
        private double rollerPercentOut;

        private Function<Double, Rotation2d> tiltFunction;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor,
                TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;
            var shooterError = parameters.turretState.shooterMotorClosedLoopError;
            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var targetAzimuth = currentAzimuth;

            if (parameters.turretState.isNoteLoaded()) {
                var robotOrientation = RobotContainer.getPoseRotation().orElse(new Rotation2d());
                var virtualGoalLocationDisplacement = parameters.turretState.virtualGoalLocationDisplacement;
                var rotationToGoal = virtualGoalLocationDisplacement.getAngle();
                var distanceToGoal = virtualGoalLocationDisplacement.getNorm();
                targetAzimuth = rotationToGoal.minus(robotOrientation);
                targetAzimuth = TurretUtils.calculateTargetAzimuthWithinLimits(targetAzimuth, currentAzimuth,
                        lowerLimit_azimuthDegrees,
                        upperLimit_azimuthDegrees);
                var rotateError = targetAzimuth.minus(currentAzimuth);
                if (Math.abs(rotateError.getDegrees()) <= tolerance.getDegrees()) {
                    outputTilt = tiltFunction.apply(distanceToGoal);
                    var tiltError = outputTilt.minus(currentElevation);
                    if (Math.abs(tiltError.getDegrees()) <= tiltTolerance.getDegrees()
                            && Math.abs(shooterError) <= shooterTolerance) {
                        rollerOn = true;
                        parameters.turretState.currentlyShooting = true;
                    }
                }
            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(targetAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));

            return StatusCode.OK;
        }

        public AimForSpeaker withTiltFunction(Function<Double, Rotation2d> tiltFunction) {
            this.tiltFunction = tiltFunction;
            return this;
        }

        public AimForSpeaker withRotateTolerance(Rotation2d tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public AimForSpeaker withTiltTolerance(Rotation2d tiltTolerance) {
            this.tiltTolerance = tiltTolerance;
            return this;
        }

        public AimForSpeaker withRollerOutput(double percentOut) {
            this.rollerPercentOut = percentOut;
            return this;

        }

        public AimForSpeaker withShooterTolerance(double shooterTolerance) {
            this.shooterTolerance = shooterTolerance;
            return this;
        }
    }

    public class AimWithRotation implements TurretRequest {
        private Rotation2d tolerance = Rotation2d.fromDegrees(1);
        private Rotation2d tilt;
        private double rollerPercentOut;
        private Rotation2d tiltTolerance = Rotation2d.fromDegrees(5);
        private double shooterTolerance;
        private Supplier<Rotation2d> rotationSupplier;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {

            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;
            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var targetAzimuth = currentAzimuth;
            var robotOrientation = RobotContainer.getPoseRotation().orElse(new Rotation2d());
            var shooterError = parameters.turretState.shooterMotorClosedLoopError;

            if (parameters.turretState.isNoteLoaded()) {
                targetAzimuth = rotationSupplier.get().minus(robotOrientation);
                targetAzimuth = TurretUtils.calculateTargetAzimuthWithinLimits(targetAzimuth, currentAzimuth,
                        lowerLimit_azimuthDegrees,
                        upperLimit_azimuthDegrees);
                var rotateError = targetAzimuth.minus(currentAzimuth);
                if (Math.abs(rotateError.getDegrees()) <= tolerance.getDegrees()) {
                    outputTilt = tilt;
                    var tiltError = outputTilt.minus(currentElevation);
                    if (Math.abs(tiltError.getDegrees()) <= tiltTolerance.getDegrees()
                            && shooterError <= shooterTolerance) {
                        rollerOn = true;
                        parameters.turretState.currentlyShooting = true;
                    }
                }
            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(targetAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));

            return StatusCode.OK;
        }

        public AimWithRotation withRotateTolerance(Rotation2d tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public AimWithRotation withTiltTolerance(Rotation2d tiltTolerance) {
            this.tiltTolerance = tiltTolerance;
            return this;
        }

        public AimWithRotation withTilt(Rotation2d tilt) {
            this.tilt = tilt;
            return this;
        }

        public AimWithRotation withRollerOutput(double percentOut) {
            this.rollerPercentOut = percentOut;
            return this;
        }

        public AimWithRotation withShooterTolerance(double shooterTolerance) {
            this.shooterTolerance = shooterTolerance;
            return this;
        }

        public AimWithRotation withRotation(Supplier<Rotation2d> rotationSupplier) {
            this.rotationSupplier = rotationSupplier;
            return this;
        }
    }

    public class ScoreAmp implements TurretRequest {
        private Rotation2d tolerance = Rotation2d.fromDegrees(1);
        private Rotation2d tilt;
        private double rollerPercentOut;
        private Rotation2d tiltTolerance = Rotation2d.fromDegrees(1);
        private Supplier<Rotation2d> rotationSupplier;
        private Supplier<Boolean> deployTriggerSupplier;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {

            var currentAzimuth = parameters.turretState.azimuth;
            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var targetAzimuth = currentAzimuth;
            var robotOrientation = RobotContainer.getPoseRotation().orElse(new Rotation2d());
            var currentElevation = parameters.turretState.elevation;

            if (parameters.turretState.isNoteLoaded()) {
                targetAzimuth = rotationSupplier.get().minus(robotOrientation);
                targetAzimuth = TurretUtils.calculateTargetAzimuthWithinLimits(targetAzimuth, currentAzimuth,
                        lowerLimit_azimuthDegrees,
                        upperLimit_azimuthDegrees);
                var azimuthError = targetAzimuth.minus(currentAzimuth);
                if (Math.abs(azimuthError.getDegrees()) <= tolerance.getDegrees()) {
                    outputTilt = tilt;
                    var tiltError = outputTilt.minus(currentElevation);
                    if (Math.abs(tiltError.getDegrees()) <= tiltTolerance.getDegrees() && deployTriggerSupplier.get()) {
                        rollerOn = true;
                        parameters.turretState.currentlyShooting = true;
                    }
                }
            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(targetAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));

            return StatusCode.OK;
        }

        public ScoreAmp withRotateTolerance(Rotation2d tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public ScoreAmp withTiltTolerance(Rotation2d tiltTolerance) {
            this.tiltTolerance = tiltTolerance;
            return this;
        }

        public ScoreAmp withTilt(Rotation2d tilt) {
            this.tilt = tilt;
            return this;
        }

        public ScoreAmp withRollerOutput(double percentOut) {
            this.rollerPercentOut = percentOut;
            return this;
        }

        public ScoreAmp withRotation(Supplier<Rotation2d> rotationSupplier) {
            this.rotationSupplier = rotationSupplier;
            return this;
        }

        public ScoreAmp withDeployTrigger(Supplier<Boolean> deployTriggerSupplier) {
            this.deployTriggerSupplier = deployTriggerSupplier;
            return this;
        }
    }

    public class ShootFromSubwoofer implements TurretRequest {

        private Supplier<Rotation2d> tiltSupplier;
        private Rotation2d rotation;
        private Supplier<Boolean> readyToShoot;
        private Rotation2d tolerance = Rotation2d.fromDegrees(3);
        private double rollerPercentOut;
        private Rotation2d tiltTolerance = Rotation2d.fromDegrees(.5);
        private double shooterTolerance;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;
            var shooterError = parameters.turretState.shooterMotorClosedLoopError;

            var rotateError = rotation.minus(currentAzimuth);
            if (Math.abs(rotateError.getDegrees()) <= tolerance.getDegrees()) {
                outputTilt = tiltSupplier.get();
                var tiltError = outputTilt.minus(currentElevation);
                if (Math.abs(tiltError.getDegrees()) <= tiltTolerance.getDegrees()
                        && Math.abs(shooterError) <= shooterTolerance && readyToShoot.get()) {
                    rollerOn = true;
                    parameters.turretState.currentlyShooting = true;
                }
            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(rotation.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));

            return StatusCode.OK;
        }

        public ShootFromSubwoofer withTilt(Supplier<Rotation2d> tiltSupplier) {
            this.tiltSupplier = tiltSupplier;
            return this;
        }

        public ShootFromSubwoofer withRotation(Rotation2d rotation) {
            this.rotation = rotation;
            return this;
        }

        public ShootFromSubwoofer withTolerance(Rotation2d tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public ShootFromSubwoofer withReadyToShoot(Supplier<Boolean> readyToShoot) {
            this.readyToShoot = readyToShoot;
            return this;
        }

        public ShootFromSubwoofer withRollerPercent(double rollerPercentOut) {
            this.rollerPercentOut = rollerPercentOut;
            return this;
        }

        public ShootFromSubwoofer withTiltTolerance(Rotation2d tiltTolerance) {
            this.tiltTolerance = tiltTolerance;
            return this;
        }

        public ShootFromSubwoofer withShooterTolerance(double shooterTolerance) {
            this.shooterTolerance = shooterTolerance;
            return this;
        }
    }

    public class Idle implements TurretRequest {
        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            rotateMotor.set(0);
            tiltMotor.set(0);
            rollerMotor.set(ControlMode.PercentOutput, 0);
            return StatusCode.OK;
        }

    }

    public class JustRoller implements TurretRequest {

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            rotateMotor.set(0);
            tiltMotor.set(0);
            rollerMotor.set(ControlMode.PercentOutput, -1.0);
            return StatusCode.OK;
        }

    }
}
