package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretRequest.calculateTargetAzimuth;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.turret.Turret.TurretState;

public interface TurretRequest {
    public class TurretControlRequestParameters {
        TurretState turretState;
    }

    public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
            TalonSRX rollerMotor);

    public class IndexFromIntake implements TurretRequest {
        private Supplier<IntakeState> intakeStateSupplier;
        private Rotation2d tolerance;
        private Rotation2d tilt;
        private double rollerPercentOut;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            var noteInPosition = intakeStateSupplier.get().noteInPosition;
            int index = -1;
            var currentAzimuth = parameters.turretState.azimuth;

            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var targetAzimuth = currentAzimuth;

            for (int i = 0; i < noteInPosition.length; i++) {
                if (noteInPosition[i]) {
                    index = i;
                    break;
                }
            }

            if (!parameters.turretState.noteLoaded && index != -1) {
                targetAzimuth = Rotation2d.fromRotations(0.25 * index);
                targetAzimuth = calculateTargetAzimuth(targetAzimuth, currentAzimuth, -350, 350);
                if (Math.abs(rotateMotor.getClosedLoopError().getValueAsDouble()) <= tolerance.getRotations()) {
                    outputTilt = tilt;
                    rollerOn = true;
                }
            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(targetAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));
            return StatusCode.OK;
        }

        public IndexFromIntake withIntakeState(Supplier<IntakeState> intakeStateSupplier) {
            this.intakeStateSupplier = intakeStateSupplier;
            return this;
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
    }

    public class AimForSpeaker implements TurretRequest {
        private Rotation2d tolerance;
        private Rotation2d tiltTolerance;
        private double rollerPercentOut;
        private Supplier<SwerveDriveState> swerveDriveStateSupplier;

        private Function<Double, Rotation2d> tiltFunction;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor,
                TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            var currentAzimuth = parameters.turretState.azimuth;
            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var targetAzimuth = currentAzimuth;

            if (parameters.turretState.noteLoaded) {
                var robotOrientation = swerveDriveStateSupplier.get().Pose.getRotation();
                var virtualGoalLocationDisplacement = parameters.turretState.virtualGoalLocationDisplacement;
                var rotationToGoal = virtualGoalLocationDisplacement.getAngle();
                var distanceToGoal = virtualGoalLocationDisplacement.getNorm();
                targetAzimuth = rotationToGoal.minus(robotOrientation);
                targetAzimuth = calculateTargetAzimuth(targetAzimuth, currentAzimuth, -350, 350);
                if (Math.abs(rotateMotor.getClosedLoopError().getValueAsDouble()) <= tolerance.getRotations()) {
                    outputTilt = tiltFunction.apply(distanceToGoal);
                    if (tiltMotor.getClosedLoopError().getValueAsDouble() <= tiltTolerance.getRotations())
                        rollerOn = true;
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

        public AimForSpeaker withSwerveDriveState(Supplier<SwerveDriveState> swerveDriveStateSupplier) {
            this.swerveDriveStateSupplier = swerveDriveStateSupplier;
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
    }

    public class AimForAmp implements TurretRequest {
        private Supplier<SwerveDriveState> swerveDriveStateSupplier;
        private Rotation2d tolerance;
        private Rotation2d tilt;
        private double rollerPercentOut;
        private Rotation2d tiltTolerance;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {

            var currentAzimuth = parameters.turretState.azimuth;
            var rollerOn = false;
            var outputTilt = new Rotation2d();
            var targetAzimuth = currentAzimuth;
            var robotOrientation = swerveDriveStateSupplier.get().Pose.getRotation();

            if (parameters.turretState.noteLoaded) {
                targetAzimuth = Rotation2d.fromDegrees(90).minus(robotOrientation);
                targetAzimuth = calculateTargetAzimuth(targetAzimuth, currentAzimuth, -350, 350);
                if (Math.abs(rotateMotor.getClosedLoopError().getValueAsDouble()) <= tolerance.getRotations()) {
                    outputTilt = tilt;
                    if (tiltMotor.getClosedLoopError().getValueAsDouble() <= tiltTolerance.getRotations())
                        rollerOn = true;
                }
            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(targetAzimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));

            return StatusCode.OK;
        }

        public AimForAmp withSwerveDriveState(Supplier<SwerveDriveState> swerveDriveStateSupplier) {
            this.swerveDriveStateSupplier = swerveDriveStateSupplier;
            return this;
        }

        public AimForAmp withRotateTolerance(Rotation2d tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public AimForAmp withTiltTolerance(Rotation2d tiltTolerance) {
            this.tiltTolerance = tiltTolerance;
            return this;
        }

        public AimForAmp withTilt(Rotation2d tilt) {
            this.tilt = tilt;
            return this;
        }

        public AimForAmp withRollerOutput(double percentOut) {
            this.rollerPercentOut = percentOut;
            return this;
        }
    }

    public class CalibrateShooter implements TurretRequest {
        private Rotation2d tolerance;
        private double rollerPercentOut;
        private Rotation2d tiltTolerance;

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

            if (parameters.turretState.noteLoaded) {
                targetAzimuth = new Rotation2d();
                if (Math.abs(rotateMotor.getClosedLoopError().getValueAsDouble()) <= tolerance.getRotations()) {
                    var checkTilt = Rotation2d.fromDegrees(SmartDashboard.getNumber("Tilt Degrees", 0));
                    if (checkTilt.getDegrees() < -50 || checkTilt.getDegrees() > 50)
                        outputTilt = checkTilt;
                    if (tiltMotor.getClosedLoopError().getValueAsDouble() <= tiltTolerance.getRotations())
                        rollerOn = true;
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

    }

    public class TestingTurret implements TurretRequest {
        private double inputPercentTilt;
        private double inputPercentRotation;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {

            rotateMotor.set(inputPercentRotation);
            tiltMotor.set(inputPercentTilt);
            return StatusCode.OK;
        }

        public TestingTurret withPercentTilt(double inputPercentTilt) {
            this.inputPercentTilt = inputPercentTilt;
            return this;
        }

        public TestingTurret withPercentRotate(double inputPercentRotation) {
            this.inputPercentRotation = inputPercentRotation;
            return this;
        }

    }

    public class ShootFromSubwoofer implements TurretRequest {

        private Rotation2d tilt;
        private Rotation2d rotation;
        private Supplier<Boolean> readyToShoot;
        private Rotation2d tolerance;
        private double rollerPercentOut;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            var rollerOn = false;
            var outputTilt = new Rotation2d();

            if (Math.abs(rotateMotor.getClosedLoopError().getValueAsDouble()) <= tolerance.getRotations()) {
                outputTilt = tilt;
                if (readyToShoot.get()) {
                    rollerOn = true;
                }
            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(rotation.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));

            return StatusCode.OK;
        }

        public ShootFromSubwoofer withTilt(Rotation2d tilt) {
            this.tilt = tilt;
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

    }

    public class Idle implements TurretRequest {
        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            return StatusCode.OK;
        }

    }

    public static Rotation2d calculateTargetAzimuth(Rotation2d target, Rotation2d current, double lowerLimitDegrees,
            double upperLimitDegrees) {
        double a = current.getDegrees();
        double b = target.getDegrees();
        a %= 360;
        a += a < 0 ? 360 : 0;
        b %= 360;
        b += b < 0 ? 360 : 0;
        double difference = b - a;
        double shiftInCurrentAngle = Math.abs(difference) < 180 ? difference
                : difference < 0 ? difference + 360 : difference - 360;
        while (shiftInCurrentAngle > upperLimitDegrees) {
            shiftInCurrentAngle -= 360;
        }
        while (shiftInCurrentAngle < lowerLimitDegrees) {
            shiftInCurrentAngle += 360;
        }
        return Rotation2d.fromDegrees(shiftInCurrentAngle);

    }

}
