package frc.robot.subsystems.intake;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeRequest.IntakeControlRequestParameters;
import frc.robot.utils.sensors.ProximitySensorInput;

public class Intake extends SubsystemBase {

    TalonSRX frontBackMotor, leftRightMotor;
    ProximitySensorInput sensors[] = new ProximitySensorInput[4];

    boolean prevNoteLoaded;

    final static double delayNoteInPositionSeconds = 0.3;

    final IntakeTelemetry intakeTelemetry = new IntakeTelemetry();

    protected IntakeRequest m_requestToApply = new IntakeRequest.Idle();
    protected IntakeControlRequestParameters m_requestParameters = new IntakeControlRequestParameters();

    private static final boolean enabled = true;
    private static Intake INSTANCE;

    public static Optional<Intake> createInstance() {
        if (INSTANCE != null) {
            return Optional.of(INSTANCE);
        }
        if (!enabled)
            return Optional.empty();
        INSTANCE = new Intake();
        return Optional.of(INSTANCE);
    }

    public static Optional<Intake> getInstance() {
        return Optional.ofNullable(INSTANCE);
    }

    private Intake() {
        frontBackMotor = new TalonSRX(19);
        leftRightMotor = new TalonSRX(20);
        leftRightMotor.setInverted(true);
        frontBackMotor.setInverted(true);
        for (int i = 0; i < sensors.length; i++) {
            sensors[i] = new ProximitySensorInput(i);
        }
    }

    public enum IntakeDirection {
        Off, In, Out;

        public static double threshold = 0.01;

        public static IntakeDirection getDirection(double percentOutput) {
            if (percentOutput > threshold)
                return In;
            else if (percentOutput < -threshold)
                return Out;
            else
                return Off;
        }
    }

    @Override
    public void periodic() {
        var fbOn = Math.abs(frontBackMotor.getMotorOutputPercent()) > 0;
        var lrOn = Math.abs(leftRightMotor.getMotorOutputPercent()) > 0;
        boolean on[] = { fbOn, lrOn, fbOn, lrOn };

        for (int i = 0; i < sensors.length && i < on.length; i++) {
            if (!m_cachedState.seeingNote[i] && sensors[i].get() && on[i]
                    && !RobotContainer.getRobotState().isActivelyGrabbing())
                m_cachedState.noteInPosition[i] = true;

            m_cachedState.seeingNote[i] = sensors[i].get();
        }

        var noteLoaded = RobotContainer.getRobotState().isNoteLoaded();
        var grabNoteIndex = RobotContainer.getRobotState().getGrabNoteIndex();
        if (!prevNoteLoaded && noteLoaded && grabNoteIndex != -1)
            m_cachedState.noteInPosition[grabNoteIndex] = false;
        prevNoteLoaded = noteLoaded;

        grabNoteIndex = -1;
        for (int i = 0; i < m_cachedState.noteInPosition.length; i++) {
            if (m_cachedState.noteInPosition[i]) {
                grabNoteIndex = i;
                break;
            }
        }

        RobotContainer.getRobotState().setGrabNoteIndex(grabNoteIndex);

        intakeTelemetry.telemeterize(m_cachedState);

        m_requestParameters.intakeState = m_cachedState;
    }

    public Command applyRequest(Supplier<IntakeRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void setControl(IntakeRequest request) {
        m_requestToApply = request;
        m_requestToApply.apply(m_requestParameters, frontBackMotor,
                leftRightMotor);
    }

    public class IntakeState {

        // 0 is Front, 1 is Left, Back is 2, Right is 3
        boolean seeingNote[] = new boolean[4];

        boolean noteInPosition[] = new boolean[4];

        public boolean[] getSeeingNote() {
            return seeingNote;
        }

        public boolean[] getNoteInPosition() {
            return noteInPosition;
        }

        public void resetAllNoteInPosition() {
            for (int i = 0; i < noteInPosition.length; i++) {
                noteInPosition[i] = false;
            }
        }
    }

    final IntakeState m_cachedState = new IntakeState();

    public void resetAllNoteInPosition() {
        m_cachedState.resetAllNoteInPosition();
    }

}
