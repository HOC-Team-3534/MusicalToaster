package frc.robot.subsystems.turret;

import java.lang.annotation.Target;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.Intake.IntakeThread;
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
        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {

            return StatusCode.OK;
        }

    }

    public class AimForAmp implements TurretRequest {

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {

            return StatusCode.OK;
        }
    }

    public class Idle implements TurretRequest {
        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            return StatusCode.OK;
        }

    }

    private static Rotation2d calculateTargetAzimuth(Rotation2d target, Rotation2d current, double lowerLimitDegrees,
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
