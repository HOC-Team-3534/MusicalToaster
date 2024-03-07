package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
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
    static final Rotation2d azimuthIndexFromIntakeTolerance = Rotation2d.fromDegrees(90);
    static final Rotation2d elevationTolerance = Rotation2d.fromDegrees(1.0);
    static final double shooterTolerance = 5.0;
    static final double rollerShootPercentOut = -1.0;

    static final double rotateQuickAccel = 0.45;
    static final double rotateSlowAccel = 0.15;
    static final double slowAccelRange = 25.0;

    public class IndexFromIntake implements TurretRequest {
        private Rotation2d tilt;
        private double rollerPercentOut;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            parameters.turretState.currentlyShooting = false;

            int index = RobotContainer.getRobotState().getGrabNoteIndex();
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

                if (MathUtils.withinTolerance(azimuthError, azimuthIndexFromIntakeTolerance)
                        && MathUtils.withinTolerance(elevationError, elevationTolerance)) {
                    RobotContainer.getRobotState().setActivelyGrabbing(true);
                    rollerOn = true;
                }
            }

            var accelRotate = MathUtils.withinTolerance(targetAzimuth.minus(currentAzimuth),
                    Rotation2d.fromDegrees(slowAccelRange))
                            ? rotateQuickAccel
                            : rotateSlowAccel;
            if (accelRotate != parameters.turretState.currentRotateAccel) {
                Constants.ROBOT.getTurretTalonConfigLiterals().getRotateConfig().MotionMagic
                        .withMotionMagicAcceleration(accelRotate);
                parameters.turretState.currentRotateAccel = accelRotate;
            }

            rotateMotor.setControl(new MotionMagicVoltage(targetAzimuth.getRotations()));
            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
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
        Function<TurretState, Optional<Rotation2d>> targetAzimuthFunction;
        Function<TurretState, Optional<Rotation2d>> targetElevationFunction;
        Supplier<Boolean> allowShootWhenAimedSupplier = () -> true;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            boolean rollerOn = false;
            Optional<Rotation2d> targetAzimuth = Optional.empty();
            Optional<Rotation2d> targetElevation = Optional.empty();

            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;

            if (parameters.turretState.isNoteLoaded() || RobotContainer.getRobotState().isClimbing()) {
                /*
                 * Azimuth Calculation
                 */
                targetAzimuth = targetAzimuthFunction.apply(parameters.turretState).map(
                        target -> TurretUtils.calculateTargetAzimuthWithinLimits(target, currentAzimuth,
                                lowerLimit_azimuthDegrees,
                                upperLimit_azimuthDegrees));

                /*
                 * Elevation Calculation
                 */
                targetElevation = targetElevationFunction.apply(parameters.turretState).map(
                        target -> RobotContainer.isTiltForcedFlat()
                                ? new Rotation2d()
                                : Rotation2d.fromDegrees(MathUtils.applyUpperAndLowerLimit(target.getDegrees(),
                                        lowerLimit_elevationDegrees, upperLimit_elevationDegrees)));

                /*
                 * Error Calculation
                 */

                if (targetAzimuth.isPresent() && targetElevation.isPresent()) {

                    var azimuthError = targetAzimuth.get().minus(currentAzimuth);
                    var elevationError = targetElevation.get().minus(currentElevation);
                    var shooterError = parameters.turretState.shooterMotorClosedLoopError;

                    if ((MathUtils.withinTolerance(azimuthError, azimuthTolerance)
                            && MathUtils.withinTolerance(elevationError, elevationTolerance)
                            && MathUtils.withinTolerance(shooterError, shooterTolerance)
                            && allowShootWhenAimedSupplier.get())
                            || parameters.turretState.currentlyShooting) {
                        rollerOn = true;
                    }
                }

            }

            var finalTargetAzimuth = targetAzimuth.orElse(currentAzimuth);

            var accelRotate = MathUtils.withinTolerance(finalTargetAzimuth.minus(currentAzimuth),
                    Rotation2d.fromDegrees(slowAccelRange))
                            ? rotateQuickAccel
                            : rotateSlowAccel;
            if (accelRotate != parameters.turretState.currentRotateAccel) {
                Constants.ROBOT.getTurretTalonConfigLiterals().getRotateConfig().MotionMagic
                        .withMotionMagicAcceleration(accelRotate);
                parameters.turretState.currentRotateAccel = accelRotate;
            }

            parameters.turretState.currentlyShooting = rollerOn;

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerShootPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(finalTargetAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(targetElevation.orElse(new Rotation2d()).getRotations()));

            return StatusCode.OK;
        }

        public ControlTurret withTargetAzimuthFunction(
                Function<TurretState, Optional<Rotation2d>> targetAzimuthFunction) {
            this.targetAzimuthFunction = targetAzimuthFunction;
            return this;
        }

        public ControlTurret withTargetElevationFunction(
                Function<TurretState, Optional<Rotation2d>> targetElevationFunction) {
            this.targetElevationFunction = targetElevationFunction;
            return this;
        }

        public ControlTurret withAllowShootWhenAimedSupplier(Supplier<Boolean> allowShootWhenAimedSupplier) {
            this.allowShootWhenAimedSupplier = allowShootWhenAimedSupplier;
            return this;
        }

    }

    public class Idle implements TurretRequest {

        VoltageOut rotateOut = new VoltageOut(0);
        VoltageOut tiltOut = new VoltageOut(0);

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            rotateMotor.setControl(rotateOut);
            tiltMotor.setControl(tiltOut);
            rollerMotor.set(ControlMode.PercentOutput, 0);
            return StatusCode.OK;
        }

    }

    public class JustRoller implements TurretRequest {

        Rotation2d tilt;
        double rollerOutput;
        Supplier<Boolean> turnOnRollerOnceReadySupplier = () -> true;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            parameters.turretState.currentlyShooting = false;

            boolean rollerOn = false;
            Rotation2d targetElevation = tilt;

            targetElevation = Rotation2d.fromDegrees(MathUtils.applyUpperAndLowerLimit(targetElevation.getDegrees(),
                    lowerLimit_elevationDegrees, upperLimit_elevationDegrees));

            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;
            if (MathUtils.withinTolerance(targetElevation, currentElevation) && turnOnRollerOnceReadySupplier.get()) {
                rollerOn = true;
                parameters.turretState.setNoteLoaded();
            }
            rotateMotor.setControl(new MotionMagicVoltage(currentAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(targetElevation.getRotations()));
            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerOutput : 0);
            return StatusCode.OK;

        }

        public JustRoller withTilt(Rotation2d tilt) {
            this.tilt = tilt;
            return this;
        }

        public JustRoller withRollerPercent(double rollerOutput) {
            this.rollerOutput = rollerOutput;
            return this;
        }

        public JustRoller withTurnOnRollerSupplier(Supplier<Boolean> turnOnRollerWhenReadySupplier) {
            this.turnOnRollerOnceReadySupplier = turnOnRollerWhenReadySupplier;
            return this;
        }

    }
}
