package frc.robot.subsystems.turret;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.Turret.TurretState;
import frc.robot.utils.TurretUtils;

public interface TurretRequest {
    public class TurretControlRequestParameters {
        TurretState turretState;
    }

    public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
            TalonSRX rollerMotor);

    public static final double lowerLimitDegrees = -240;
    public static final double upperLimitDegrees = 240;

    public class IndexFromIntake implements TurretRequest {
        private Rotation2d tolerance = Rotation2d.fromDegrees(3);
        private Rotation2d tilt;
        private double rollerPercentOut;
        private Rotation2d tiltTolerance = Rotation2d.fromDegrees(3);

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            ;
            int index = RobotContainer.getRobotState().grabNoteIndex;
            var currentAzimuth = parameters.turretState.azimuth;
            var currentElevation = parameters.turretState.elevation;
            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var targetAzimuth = currentAzimuth;

            if (!parameters.turretState.isNoteLoaded() && index != -1) {
                targetAzimuth = Rotation2d.fromRotations(0.25 * (index + 2));
                targetAzimuth = TurretUtils.calculateTargetAzimuth(targetAzimuth, currentAzimuth, lowerLimitDegrees,
                        upperLimitDegrees);
                var rotateError = targetAzimuth.minus(currentAzimuth);
                if (Math.abs(rotateError.getDegrees()) <= tolerance.getDegrees()
                        && !RobotContainer.isTiltForcedFlat()) {
                    outputTilt = tilt;
                    var tiltError = outputTilt.minus(currentElevation);
                    if (Math.abs(tiltError.getDegrees()) <= tiltTolerance.getDegrees()) {
                        RobotContainer.setActivelyGrabbing(true);
                        rollerOn = true;
                    }
                }
            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(targetAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));
            return StatusCode.OK;
        }

        public IndexFromIntake withRotateTolerance(Rotation2d tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public IndexFromIntake withTilt(Rotation2d tilt) {
            this.tilt = tilt;
            return this;
        }

        public IndexFromIntake withRollerOutput(double percentOut) {
            this.rollerPercentOut = percentOut;
            return this;
        }

        public IndexFromIntake withTiltTolerance(Rotation2d tiltTolerance) {
            this.tiltTolerance = tiltTolerance;
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
                targetAzimuth = TurretUtils.calculateTargetAzimuth(targetAzimuth, currentAzimuth, lowerLimitDegrees,
                        upperLimitDegrees);
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
                targetAzimuth = TurretUtils.calculateTargetAzimuth(targetAzimuth, currentAzimuth, lowerLimitDegrees,
                        upperLimitDegrees);
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
                targetAzimuth = TurretUtils.calculateTargetAzimuth(targetAzimuth, currentAzimuth, lowerLimitDegrees,
                        upperLimitDegrees);
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

    public class CalibrateShooter implements TurretRequest {
        private Rotation2d tolerance = Rotation2d.fromDegrees(1);
        private double rollerPercentOut;
        private Rotation2d tiltTolerance = Rotation2d.fromDegrees(1);
        private double shooterTolerance;

        public CalibrateShooter() {
            SmartDashboard.putNumber("Tilt Degrees", 0);
        }

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            var currentAzimuth = parameters.turretState.azimuth;
            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var targetAzimuth = currentAzimuth;
            var currentElevation = parameters.turretState.elevation;
            var shooterError = parameters.turretState.shooterMotorClosedLoopError;

            if (parameters.turretState.isNoteLoaded()) {
                targetAzimuth = new Rotation2d();
                var rotateError = targetAzimuth.minus(currentAzimuth);
                if (Math.abs(rotateError.getDegrees()) <= tolerance.getDegrees()) {
                    var checkTilt = Rotation2d.fromDegrees(SmartDashboard.getNumber("Tilt Degrees", 0));
                    if (checkTilt.getDegrees() < -50 || checkTilt.getDegrees() > 50)
                        outputTilt = checkTilt;
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

        public CalibrateShooter withRotateTolerance(Rotation2d tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public CalibrateShooter withTiltTolerance(Rotation2d tiltTolerance) {
            this.tiltTolerance = tiltTolerance;
            return this;
        }

        public CalibrateShooter withRollerOutput(double percentOut) {
            this.rollerPercentOut = percentOut;
            return this;
        }

        public CalibrateShooter withShooterTolerance(double shooterTolerance) {
            this.shooterTolerance = shooterTolerance;
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
