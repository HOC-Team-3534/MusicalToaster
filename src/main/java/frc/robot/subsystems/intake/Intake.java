package frc.robot.subsystems.intake;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    TalonSRX frontBackMotor, leftRightMotor;

    ReadWriteLock stateLock = new ReentrantReadWriteLock();
    protected IntakeRequest requestToApply = new IntakeRequest.Idle();

    public Intake() {
        frontBackMotor = new TalonSRX(0);
        leftRightMotor = new TalonSRX(0);
    }

    public Command applyRequest(Supplier<IntakeRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void setControl(IntakeRequest request) {
        try {
            stateLock.writeLock().lock();

            requestToApply = request;
        } finally {
            stateLock.writeLock().unlock();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public class IntakeThread {
        final Thread thread;
        volatile boolean running;

        public IntakeThread() {
            thread = new Thread(this::run);

            thread.setDaemon(true);
        }

        public void run() {

        }
    }

}
