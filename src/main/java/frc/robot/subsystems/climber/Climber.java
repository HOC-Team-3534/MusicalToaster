package frc.robot.subsystems.climber;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberRequest.ControlClimberRequestParameters;
import frc.robot.subsystems.intake.Intake;

public class Climber extends SubsystemBase {
    TalonFX climberMotor;

    final double UpdateFrequency = 50.0;

    protected ClimberRequest m_requestToApply = new ClimberRequest.Idle();
    protected ControlClimberRequestParameters m_requestParameters = new ControlClimberRequestParameters();
    private static final boolean enabled = true;
    private static Climber INSTANCE;

    public static Optional<Climber> createInstance() {
        if (INSTANCE != null) {
            return Optional.of(INSTANCE);
        }
        if (!enabled)
            return Optional.empty();
        INSTANCE = new Climber();
        return Optional.of(INSTANCE);
    }

    public static Optional<Climber> getInstance() {
        return Optional.ofNullable(INSTANCE);
    }

    private Climber() {
        climberMotor = new TalonFX(21);

        TalonFXConfiguration cfgClimber = new TalonFXConfiguration();

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
