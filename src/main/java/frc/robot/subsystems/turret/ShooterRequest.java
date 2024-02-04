package frc.robot.subsystems.turret;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.turret.TurretRequest.TurretControlRequestParameters;

public interface ShooterRequest {
    public StatusCode apply(TurretControlRequestParameters parameters, TalonFX rightShooterMotor);

    public class ControlShooter implements ShooterRequest {
        private double voltage = 0;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters,
                TalonFX rightShooterMotor) {
            rightShooterMotor.setControl(new VoltageOut(voltage));
            return StatusCode.OK;
        }

        public ControlShooter withVoltage(double voltage) {
            this.voltage = voltage;
            return this;
        }

    }

    public class Idle implements ShooterRequest {

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters,
                TalonFX rightShooterMotor) {
            return StatusCode.OK;
        }
    }
}
