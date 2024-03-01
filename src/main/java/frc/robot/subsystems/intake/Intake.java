package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemThread;
import frc.robot.subsystems.intake.IntakeRequest.IntakeControlRequestParameters;
import frc.robot.utils.sensors.ProximitySensorInput;

public class Intake extends SubsystemBase {

    TalonSRX frontBackMotor, leftRightMotor;
    ProximitySensorInput sensors[] = new ProximitySensorInput[4];

    final double UpdateFrequency = 50.0;

    final static double delayNoteInPositionSeconds = 0.3;

    private final SubsystemThread m_thread = new SubsystemThread(UpdateFrequency) {

        boolean prevNoteLoaded;

        @Override
        public void run() {
            var fbPerc = frontBackMotor.getMotorOutputPercent();
            var lrPerc = leftRightMotor.getMotorOutputPercent();
            double percs[] = { fbPerc, lrPerc, fbPerc, lrPerc };

            for (int i = 0; i < percs.length; i++) {
                m_cachedState.intakeDirection[i] = IntakeDirection.getDirection(percs[i]);
            }

            for (int i = 0; i < sensors.length; i++) {
                /*
                 * If the note is not already marked in position
                 * and the note is seen while the motor is running in
                 * set note in position to true and reset the delay timer
                 */
                if (!m_cachedState.seeingNote[i]
                        && sensors[i].get()) {
                    m_cachedState.noteInPosition[i] = true;
                }

                m_cachedState.seeingNote[i] = sensors[i].get();

                if (resetNoteSupplier.get()) {
                    m_cachedState.noteInPosition[i] = false;
                }
            }
            var grabNoteIndex = -1;
            for (int i = 0; i < m_cachedState.noteInPosition.length; i++) {
                if (m_cachedState.noteInPosition[i]) {
                    grabNoteIndex = i;
                    break;
                }
            }

            RobotContainer.setGrabNoteIndex(grabNoteIndex);

            var noteLoaded = RobotContainer.getRobotState().noteLoaded;

            if (!prevNoteLoaded && noteLoaded && grabNoteIndex != -1) {
                m_cachedState.noteInPosition[grabNoteIndex] = false;
            }

            prevNoteLoaded = noteLoaded;

            // intakeTelemetry.telemeterize(m_cachedState);

            m_requestParameters.intakeState = m_cachedState;

            m_requestToApply.apply(m_requestParameters, frontBackMotor,
                    leftRightMotor);
        }

    };

    final IntakeTelemetry intakeTelemetry = new IntakeTelemetry();

    protected IntakeRequest m_requestToApply = new IntakeRequest.Idle();
    protected IntakeControlRequestParameters m_requestParameters = new IntakeControlRequestParameters();

    private final Supplier<Boolean> resetNoteSupplier;

    public Intake(Supplier<Boolean> resetNoteSupplier) {
        frontBackMotor = new TalonSRX(19);
        leftRightMotor = new TalonSRX(20);
        leftRightMotor.setInverted(true);
        frontBackMotor.setInverted(true);
        for (int i = 0; i < sensors.length; i++) {
            sensors[i] = new ProximitySensorInput(i);
        }

        m_thread.start();
        this.resetNoteSupplier = resetNoteSupplier;
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

    public Command applyRequest(Supplier<IntakeRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void setControl(IntakeRequest request) {
        try {
            m_thread.writeLock().lock();

            m_requestToApply = request;
        } finally {
            m_thread.writeLock().unlock();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public class IntakeState {
        public IntakeState() {
            for (int i = 0; i < intakeDirection.length; i++) {
                intakeDirection[i] = IntakeDirection.Off;
            }
        }

        // 0 is Front, 1 is Left, Back is 2, Right is 3
        public boolean seeingNote[] = new boolean[4];

        public boolean noteInPosition[] = new boolean[4];

        public IntakeDirection intakeDirection[] = new IntakeDirection[4];
    }

    final IntakeState m_cachedState = new IntakeState();

}
