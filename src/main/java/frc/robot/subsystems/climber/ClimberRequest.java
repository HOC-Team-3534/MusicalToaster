package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

public interface ClimberRequest {
    public class ControlClimberRequestParameters {
    }

    public StatusCode apply(ControlClimberRequestParameters parameters, TalonFX climberMotor);

    public class ControlClimber implements ClimberRequest {
        public double percent;
        public boolean isReversed;

        @Override
        public StatusCode apply(ControlClimberRequestParameters parameters, TalonFX climberMotor) {
            var percentOut = isReversed ? percent : percent * -1;
            climberMotor.set(percentOut);

            return StatusCode.OK;
        }

        public ControlClimber withVoltage(double percent) {
            this.percent = percent;
            return this;
        }

        public ControlClimber withClimberReversed(boolean isReversed) {
            this.isReversed = isReversed;
            return this;
        }

    }

    /*
     * Next commented code was start for pull up climber, builders said only to have
     * a button for climbing.
     */

    // public class PullUpClimber implements ClimberRequest {
    // public double voltage;
    // public boolean isReversed;

    // @Override
    // public StatusCode apply(TurretControlRequestParameters parameters, TalonFX
    // climberMotor) {

    // return StatusCode.OK;
    // }

    // public ClimberRequest withVoltage(double voltage) {
    // this.voltage = voltage;
    // return this;
    // }

    // public ClimberRequest withReversal(boolean isReversed) {
    // this.isReversed = isReversed;
    // return this;
    // }

    // }

    public class Idle implements ClimberRequest {

        @Override
        public StatusCode apply(ControlClimberRequestParameters parameters,
                TalonFX rightClimberMotor) {
            return StatusCode.OK;
        }
    }
}
