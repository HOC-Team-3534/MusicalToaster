package frc.robot.subsystems.climber;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberRequest.ControlClimberRequestParameters;

public class Climber extends SubsystemBase {
    TalonFX climberMotor;

    final double UpdateFrequency = 50.0;

    protected ClimberRequest m_requestToApply = new ClimberRequest.Idle();
    protected ControlClimberRequestParameters m_requestParameters = new ControlClimberRequestParameters();

    public Climber() {
        climberMotor = new TalonFX(21);

        TalonFXConfiguration cfgClimber = new TalonFXConfiguration();

        MotionMagicConfigs mmClimber = cfgClimber.MotionMagic;
        mmClimber.MotionMagicCruiseVelocity = .5;
        mmClimber.MotionMagicAcceleration = 5;
        mmClimber.MotionMagicJerk = 30;

        Slot0Configs slot0Climber = cfgClimber.Slot0;
        slot0Climber.kP = 10;
        slot0Climber.kI = 0;
        slot0Climber.kV = 0;// TODO Tune these values
        slot0Climber.kS = 0;

        Slot1Configs slot1Climber = cfgClimber.Slot1;
        slot1Climber.kP = 10;
        slot1Climber.kI = 0;
        slot1Climber.kV = 0;
        slot1Climber.kS = 0;

        FeedbackConfigs fdbClimber = cfgClimber.Feedback;
        fdbClimber.SensorToMechanismRatio = 125;

        StatusCode statusClimber = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; i++)
            statusClimber = climberMotor.getConfigurator().apply(cfgClimber);
        if (!statusClimber.isOK())
            System.out.println("Could not configure device. Error: " + statusClimber.toString());

        climberMotor.setPosition(0);
    }

    public Command applyRequest(
            Supplier<ClimberRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void setControl(ClimberRequest request) {
        m_requestToApply = request;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public class ClimberState {
        public double climberPosition;
    }

    final ClimberState m_cachedState = new ClimberState();
}
