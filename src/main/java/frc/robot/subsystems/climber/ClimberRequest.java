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

    static final double stopRotations1 = 12.5;
    static final double stopRotations2 = 20.47;
    static final double gapTime = 0.25;

    public StatusCode apply(ControlClimberRequestParameters parameters, TalonFX climberMotor);

    public class ControlClimber implements ClimberRequest {
        double voltage;
        VoltageOut voltageOut = new VoltageOut(0);
        Timer timer = new Timer();
        boolean onPart2 = false;

        @Override
        public StatusCode apply(ControlClimberRequestParameters parameters, TalonFX climberMotor) {

            var position = parameters.climberState.climberPosition;

            if (position >= stopRotations1 && timer.hasElapsed(gapTime)) {
                onPart2 = true;
            }

            var climberOn = position < stopRotations1
                    || (onPart2 && position >= stopRotations1 && position < stopRotations2);

            climberMotor.setControl(voltageOut.withOutput(climberOn ? voltage : 0));
            timer.restart();
            return StatusCode.OK;
        }

        public ControlClimber withVoltage(double voltage) {
            this.voltage = voltage;
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
