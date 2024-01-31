package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;

public interface IntakeRequest {
    public class IntakeControlRequestParameters {
        boolean leftRightHasNote, frontBackHasNote;

    }

    public StatusCode apply(IntakeControlRequestParameters parameters, TalonSRX frontBack, TalonSRX leftRight);

    public class ControlIntake implements IntakeRequest {

        private boolean run;
        private boolean reversed;
        private double intakePercent;

        @Override
        public StatusCode apply(IntakeControlRequestParameters parameters, TalonSRX frontBack, TalonSRX leftRight) {
            var frontBackPercentOut = run ? reversed || parameters.leftRightHasNote ? intakePercent * -1 : intakePercent
                    : 0;

            var leftRightPercentOut = run ? reversed || parameters.frontBackHasNote ? intakePercent * -1 : intakePercent
                    : 0;
            frontBack.set(ControlMode.PercentOutput, frontBackPercentOut);
            leftRight.set(ControlMode.PercentOutput, leftRightPercentOut);
            return StatusCode.OK;
        }

        public ControlIntake withIntakeRunning(boolean run) {
            this.run = run;
            return this;
        }

        public ControlIntake withIntakeReversed(boolean reversed) {
            this.reversed = reversed;
            return this;
        }

        public ControlIntake withIntakePercent(double intakePercent) {
            this.intakePercent = intakePercent;
            return this;
        }
    }

}
