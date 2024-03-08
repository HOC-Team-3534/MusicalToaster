package frc.robot.subsystems.climber;

import java.util.Optional;
import java.util.function.Function;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.Climber.ClimberState;

public interface ClimberRequest {
    public class ControlClimberRequestParameters {
        ClimberState climberState;
    }

    static final double stopRotations1 = 13.1;
    static final double stopRotations2 = 20.76;
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

    public class ResetClimber implements ClimberRequest {

        Function<Double, Double> voltageFunction = (rotationsMoved) -> {
            if (rotationsMoved > 0.05)
                return 0.0;
            if (rotationsMoved > 0.75 * -stopRotations2)
                return -12.0;
            if (rotationsMoved > 0.9 * -stopRotations2)
                return -4.0;
            if (rotationsMoved > -stopRotations2)
                return -2.5;
            return 0.0;
        };

        VoltageOut voltageOut = new VoltageOut(0);

        Optional<Double> startPosition = Optional.empty();

        @Override
        public StatusCode apply(ControlClimberRequestParameters parameters, TalonFX climberMotor) {
            if (!DriverStation.isFMSAttached()) {
                var position = parameters.climberState.climberPosition;

                if (startPosition.isEmpty()) {
                    startPosition = Optional.of(position);
                }

                var rotationsMoved = position - startPosition.get();

                if (rotationsMoved <= 0.95 * -stopRotations2) {
                    RobotContainer.getRobotState().resetResetingClimber();
                    climberMotor.setControl(voltageOut.withOutput(0));
                } else {
                    RobotContainer.getRobotState().setResetingClimber();
                    climberMotor.setControl(voltageOut.withOutput(voltageFunction.apply(rotationsMoved)));
                }
                return StatusCode.OK;
            } else {
                return new Idle().apply(parameters, climberMotor);
            }
        }

        public ResetClimber withVoltageFunction(Function<Double, Double> voltageFunction) {
            this.voltageFunction = voltageFunction;
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
