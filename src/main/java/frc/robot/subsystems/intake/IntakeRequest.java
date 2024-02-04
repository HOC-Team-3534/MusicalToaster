package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;

import frc.robot.subsystems.intake.Intake.IntakeState;

public interface IntakeRequest {
    public class IntakeControlRequestParameters {
        IntakeState intakeState;
    }

    public StatusCode apply(IntakeControlRequestParameters parameters, TalonSRX frontBack, TalonSRX leftRight);

    public class ControlIntake implements IntakeRequest {
        private boolean reversed;
        private double intakePercent;

        @Override
        public StatusCode apply(IntakeControlRequestParameters parameters, TalonSRX frontBack, TalonSRX leftRight) {
            var inPosition = parameters.intakeState.noteInPosition;
            var seeNote = parameters.intakeState.seeingNote;
            var dontRunFrontBack = seeNote[1] || seeNote[3] || inPosition[1] || inPosition[3];
            var dontRunLeftRight = seeNote[0] || seeNote[2] || inPosition[0] || inPosition[2];

            // TODO Automate extake on multiple notes

            var frontBackPercentOut = reversed ? intakePercent * -1 : dontRunFrontBack ? 0 : intakePercent;
            var leftRightPercentOut = reversed ? intakePercent * -1 : dontRunLeftRight ? 0 : intakePercent;
            frontBack.set(ControlMode.PercentOutput, frontBackPercentOut);
            leftRight.set(ControlMode.PercentOutput, leftRightPercentOut);

            return StatusCode.OK;
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
