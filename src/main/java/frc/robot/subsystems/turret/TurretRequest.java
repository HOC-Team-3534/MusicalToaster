package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.Turret.TurretState;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TurretUtils;
import frc.robot.utils.sensors.SensorMonitor;

public interface TurretRequest {
    public class TurretControlRequestParameters {
        TurretState turretState;
    }

    public StatusCode apply(TurretControlRequestParameters parameters, Consumer<Rotation2d> rotateTurretConsumer,
            Consumer<Rotation2d> tiltTurretConsumer,
            TalonSRX rollerMotor);

    static final double lowerLimit_azimuthDegrees = -240;
    static final double upperLimit_azimuthDegrees = 240;

    static final double lowerLimit_elevationDegrees = -50;
    static final double upperLimit_elevationDegrees = 50;

    static final Rotation2d azimuthTolerance = Rotation2d.fromDegrees(3.0);
    static final Rotation2d azimuthIndexFromIntakeTolerance = Rotation2d.fromDegrees(70);
    static final Rotation2d elevationTolerance = Rotation2d.fromDegrees(0.75);
    static final Rotation2d elevationWhenRotating = Rotation2d.fromDegrees(30.0);
    static final Rotation2d wideAzimuthToleranceForTilt = Rotation2d.fromDegrees(25);
    static final Rotation2d stillAzimuthTolerance = azimuthTolerance.plus(Rotation2d.fromDegrees(3));
    static final Rotation2d stillElevationTolerance = elevationTolerance.plus(Rotation2d.fromDegrees(5));

    static final double shooterTolerance = 5.0;
    static final double rollerShootPercentOut = -1.0;

    public class IndexFromIntake implements TurretRequest {
        private Rotation2d tilt;
        private double rollerPercentOut;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, Consumer<Rotation2d> rotateTurretConsumer,
                Consumer<Rotation2d> tiltTurretConsumer,
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

                outputTilt = RobotState.isTiltForcedFlat()
                        ? new Rotation2d()
                        : tilt;

                var azimuthError = MathUtils.firstMinusSecondRotation(targetAzimuth, currentAzimuth);
                var elevationError = MathUtils.firstMinusSecondRotation(outputTilt, currentElevation);

                if (MathUtils.withinTolerance(azimuthError, azimuthIndexFromIntakeTolerance)
                        && MathUtils.withinTolerance(elevationError, elevationTolerance)) {
                    RobotContainer.getRobotState().setActivelyGrabbing(true);
                    rollerOn = true;
                }
            }

            rotateTurretConsumer.accept(targetAzimuth);
            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            tiltTurretConsumer.accept(outputTilt);
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
        Supplier<Boolean> shootImmediatelyOverride = () -> false;

        private SensorMonitor onTargetMonitor = new SensorMonitor(0.05, 0.02, 0.01);

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, Consumer<Rotation2d> rotateTurretConsumer,
                Consumer<Rotation2d> tiltTurretConsumer,
                TalonSRX rollerMotor) {
            boolean rollerOn = false;
            Optional<Rotation2d> targetAzimuth = Optional.empty();
            Optional<Rotation2d> targetElevation = Optional.empty();

            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;

            if (parameters.turretState.isNoteLoaded() || RobotContainer.getRobotState().isClimbing()
                    || RobotContainer.getRobotState().isExtaking()) {
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
                        target -> RobotState.isTiltForcedFlat()
                                ? new Rotation2d()
                                : Rotation2d.fromDegrees(MathUtils.applyUpperAndLowerLimit(target.getDegrees(),
                                        lowerLimit_elevationDegrees, upperLimit_elevationDegrees)));

                /*
                 * Error Calculation
                 */

                if (targetAzimuth.isPresent() && targetElevation.isPresent()) {

                    SmartDashboard.putNumber("Rotate Target Azimuth", targetAzimuth.get().getDegrees());

                    var azimuthError = MathUtils.firstMinusSecondRotation(targetAzimuth.get(), currentAzimuth);
                    var elevationError = MathUtils.firstMinusSecondRotation(targetElevation.get(), currentElevation);
                    var shooterError = parameters.turretState.shooterMotorClosedLoopError;

                    if (!MathUtils.withinTolerance(azimuthError, wideAzimuthToleranceForTilt)
                            && targetElevation.get().getDegrees() > elevationWhenRotating.getDegrees()) {
                        targetElevation = Optional.of(elevationWhenRotating);
                    }

                    var withinTolerance = MathUtils.withinTolerance(azimuthError, azimuthTolerance)
                            && MathUtils.withinTolerance(elevationError, elevationTolerance)
                            && MathUtils.withinTolerance(shooterError, shooterTolerance);

                    var withinStillTolerance = MathUtils.withinTolerance(azimuthError, azimuthTolerance)
                            && MathUtils.withinTolerance(elevationError, elevationTolerance)
                            && parameters.turretState.hasTurretAnglesBeenStill();

                    var goodToShoot = (withinTolerance || withinStillTolerance)
                            && allowShootWhenAimedSupplier.get();

                    onTargetMonitor.addSensorValue(goodToShoot ? 1 : 0);

                    if ((!onTargetMonitor.hasSignificantMovement() && goodToShoot) || shootImmediatelyOverride.get()
                            || parameters.turretState.currentlyShooting) {
                        rollerOn = true;
                    }
                }

            }

            parameters.turretState.currentlyShooting = rollerOn;

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerShootPercentOut : 0);
            rotateTurretConsumer.accept(targetAzimuth.orElse(currentAzimuth));
            tiltTurretConsumer.accept(targetElevation.orElse(new Rotation2d()));

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

        public ControlTurret withShootImmediatelyOverrideSupplier(Supplier<Boolean> shootImmediatelyOverride) {
            this.shootImmediatelyOverride = shootImmediatelyOverride;
            return this;
        }

    }

    public class Idle implements TurretRequest {

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, Consumer<Rotation2d> rotateTurretConsumer,
                Consumer<Rotation2d> tiltTurretConsumer,
                TalonSRX rollerMotor) {
            rotateTurretConsumer.accept(null);
            tiltTurretConsumer.accept(null);
            rollerMotor.set(ControlMode.PercentOutput, 0);
            return StatusCode.OK;
        }

    }

    public class JustRoller implements TurretRequest {

        Rotation2d tilt;
        double rollerOutput;
        Function<Boolean, Boolean> turnOnRollerOnceReadyFunction = (withinTolerance) -> true;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, Consumer<Rotation2d> rotateTurretConsumer,
                Consumer<Rotation2d> tiltTurretConsumer,
                TalonSRX rollerMotor) {
            parameters.turretState.currentlyShooting = false;

            boolean rollerOn = false;
            Rotation2d targetElevation = tilt;

            targetElevation = Rotation2d.fromDegrees(MathUtils.applyUpperAndLowerLimit(targetElevation.getDegrees(),
                    lowerLimit_elevationDegrees, upperLimit_elevationDegrees));

            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;
            if (turnOnRollerOnceReadyFunction.apply(MathUtils.withinTolerance(targetElevation, currentElevation))) {
                rollerOn = true;
                parameters.turretState.setNoteLoaded();
            }
            rotateTurretConsumer.accept(currentAzimuth);
            tiltTurretConsumer.accept(targetElevation);
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

        public JustRoller withTurnOnRollerSupplier(Function<Boolean, Boolean> turnOnRollerWhenReadyFunction) {
            this.turnOnRollerOnceReadyFunction = turnOnRollerWhenReadyFunction;
            return this;
        }

    }
}
