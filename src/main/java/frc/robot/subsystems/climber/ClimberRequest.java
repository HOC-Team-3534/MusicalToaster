package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.climber.Climber.ClimberState;

public interface ClimberRequest {
    public class ControlClimberRequestParameters {
        ClimberState climberState;
    }

    static final double stopRotations1 = 1.0;
    static final double stopRotations2 = 2.0;
    static final double gapTime = 0.25;

    public StatusCode apply(ControlClimberRequestParameters parameters, TalonFX climberMotor);

    public class ControlClimber implements ClimberRequest {
        double percent;
        VoltageOut voltage = new VoltageOut(0);
        Timer timer = new Timer();

        @Override
        public StatusCode apply(ControlClimberRequestParameters parameters, TalonFX climberMotor) {
            var position = parameters.climberState.climberPosition;

            var climberOn = position < stopRotations1
                    || (position >= stopRotations1 && position < stopRotations2 && timer.hasElapsed(gapTime));

            climberMotor.setControl(voltage.withOutput(climberOn ? percent : 0));
            timer.restart();
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
