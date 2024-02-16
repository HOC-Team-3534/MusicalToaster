package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.turret.TurretRequest.TurretControlRequestParameters;

public interface ClimberRequest {
    public StatusCode apply(TurretControlRequestParameters parameters, TalonFX climberMotor);

    public class ControlClimber implements ClimberRequest {
        private double velocity = 0;

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters,
                TalonFX rightClimberMotor) {
            rightClimberMotor.setControl(new VelocityVoltage(velocity));
            return StatusCode.OK;
        }

        public ControlClimber withVelocity(double velocity) {
            this.velocity = velocity;
            return this;
        }

    }

    public class Idle implements ClimberRequest {

        @Override
        public StatusCode apply(TurretControlRequestParameters parameters,
                TalonFX rightClimberMotor) {
            return StatusCode.OK;
        }
    }
}
