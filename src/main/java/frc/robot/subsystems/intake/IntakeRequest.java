package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;

public interface IntakeRequest {
    public class IntakeControlRequestParameters {

    }

    public StatusCode apply(IntakeControlRequestParameters parameters, TalonSRX frontBack, TalonSRX leftRight);

    public class ControlIntake implements IntakeRequest {

        private boolean run;
        private boolean reversed;
        private double intakePercent;
        private boolean leftRightHasNote;
        private boolean frontBackHasNote;

        @Override
        public StatusCode apply(IntakeControlRequestParameters parameters, TalonSRX frontBack, TalonSRX leftRight) {
            var frontBackPercentOut = run ? reversed || leftRightHasNote ? intakePercent * -1 : intakePercent
                    : 0;
            var leftRightPercentOut = run ? reversed || frontBackHasNote ? intakePercent * -1 : intakePercent
                    : 0;
            frontBack.set(ControlMode.PercentOutput, frontBackPercentOut);
            leftRight.set(ControlMode.PercentOutput, leftRightPercentOut);
            return StatusCode.OK;
        }

        public ControlIntake withFronBackHasNote(boolean hasNote) {
            this.frontBackHasNote = hasNote;
            return this;
        }

        public ControlIntake withLeftRightHasNote(boolean hasNote) {
            this.leftRightHasNote = hasNote;
            return this;
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

    public class Idle implements IntakeRequest {
        @Override
        public StatusCode apply(IntakeControlRequestParameters parameters, TalonSRX frontBack, TalonSRX leftRight) {
            return StatusCode.OK;
        }

    }

}
