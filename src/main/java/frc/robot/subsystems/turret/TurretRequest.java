package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.turret.Turret.TurretState;

public interface TurretRequest {
    public class TurretControlRequestParameters {
        TurretState turretState;
    }

    public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
            TalonSRX rollerMotor);

    public class IndexFromIntake implements TurretRequest {
        private IntakeState intakeState;
        private Rotation2d tolerance;
        private Rotation2d tilt;
        private double rollerPercentOut;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rotateMotor, TalonFX tiltMotor,
                TalonSRX rollerMotor) {
            var noteInPosition = intakeState.noteInPosition;
            int index = -1;
            var azimuth = parameters.turretState.azimuth;
            var rollerOn = false;

            var outputTilt = new Rotation2d();
            for (int i = 0; i < noteInPosition.length; i++) {
                if (noteInPosition[i]) {
                    index = i;
                    break;
                }

            }
            if (!parameters.turretState.noteLoaded) {
                azimuth = Rotation2d.fromRotations(0.25 * index);
                if (Math.abs(rotateMotor.getClosedLoopError().getValueAsDouble()) <= tolerance.getRotations()) {
                    outputTilt = tilt;
                    rollerOn = true;
                }

            }

            rollerMotor.set(ControlMode.PercentOutput, rollerOn ? rollerPercentOut : 0);
            rotateMotor.setControl(new MotionMagicVoltage(azimuth.getRotations()));
            tiltMotor.setControl(new MotionMagicVoltage(outputTilt.getRotations()));
            return StatusCode.OK;
        }

        public IndexFromIntake withIntakeState(IntakeState intakeState) {
            this.intakeState = intakeState;
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

        public IndexFromIntake withRollerVoltage(double percentOut) {
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

}
