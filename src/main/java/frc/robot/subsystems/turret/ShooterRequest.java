package frc.robot.subsystems.turret;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.turret.TurretRequest.TurretControlRequestParameters;

public interface ShooterRequest {
    public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rightShooterMotor);

    public class ControlShooter implements ShooterRequest {
        private double velocity = 0;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters,
                TalonFX rightShooterMotor) {
            rightShooterMotor.setControl(new VelocityVoltage(velocity));
            return StatusCode.OK;
        }

        public ControlShooter withVelocity(double velocity) {
            this.velocity = velocity;
            return this;
        }

    }

    public class ControlShooterPercentage implements ShooterRequest {
        private double percentage = 0;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters,
                TalonFX rightShooterMotor) {
            rightShooterMotor.setControl(new VoltageOut(percentage * 12));
            return StatusCode.OK;
        }

        public ControlShooterPercentage withPercentOut(double percentage) {
            this.percentage = percentage;
            return this;
        }
    }

    public class Idle implements ShooterRequest {

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters,
                TalonFX rightShooterMotor) {
            rightShooterMotor.setControl(new VoltageOut(0));
            return StatusCode.OK;
        }
    }
}
