package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public interface ClimberRequest {
    public class ControlClimberRequestParameters {
    }

    public StatusCode apply(ControlClimberRequestParameters parameters, TalonFX climberMotor);

    public class ControlClimber implements ClimberRequest {
        double percent;
        VoltageOut voltage = new VoltageOut(0);

        @Override
        public StatusCode apply(ControlClimberRequestParameters parameters, TalonFX climberMotor) {
            climberMotor.setControl(voltage.withOutput(percent));
            return StatusCode.OK;
        }

        public ControlClimber withVoltage(double percent) {
            this.percent = percent;
            return this;
        }

    }

    public class Idle implements ClimberRequest {
        VoltageOut voltage = new VoltageOut(0);

        @Override
        public StatusCode apply(ControlClimberRequestParameters parameters,
                TalonFX climberMotor) {
            climberMotor.setControl(voltage);
            return StatusCode.OK;
        }
    }
}
